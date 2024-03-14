#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

#include "SerialDevice.hpp"
#include "Quori.hpp"
#include "Csv.hpp"

#include <memory>
#include <fstream>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using namespace quori_controller;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("quori_controller");
    rclcpp::executors::MultiThreadedExecutor executor;

    std::vector<std::string> devices;
    node->declare_parameter<std::vector<std::string>>("devices", std::vector<std::string>());
    if (!node->get_parameter("devices", devices)) {
        RCLCPP_ERROR(node->get_logger(), "Expected 'devices' parameter.");
        return EXIT_FAILURE;
    }

    bool generate_csvs = false;
    node->declare_parameter<bool>("generate_csvs", false);
    node->get_parameter("generate_csvs", generate_csvs);

    boost::asio::io_service service;
    std::vector<SerialDevice::Ptr> serial_devices;
    std::unique_ptr<std::ofstream> csv_file;
    Csv::Ptr csv;

    if (generate_csvs) {
        csv_file = std::make_unique<std::ofstream>("out.csv");
        if (!*csv_file) {
            RCLCPP_ERROR(node->get_logger(), "Could not open CSV file.");
            return EXIT_FAILURE;
        }
        csv = Csv::open(*csv_file);
        RCLCPP_INFO(node->get_logger(), "Opened CSV file.");
    }

    for (const auto& device : devices) {
        const auto serial_device = SerialDevice::open(service, device);
        if (csv) serial_device->attachSetPositionsCsv(csv);
        serial_devices.push_back(serial_device);
    }

    Quori robot(node, serial_devices);
    controller_manager::ControllerManager cm(&robot, node);

    double rate_hz = 100.0;
    node->declare_parameter<double>("rate", 100.0);
    node->get_parameter("rate", rate_hz);

    rclcpp::Time last_read = node->get_clock()->now();
    rclcpp::Time last_update = node->get_clock()->now();
    rclcpp::Time last_write = node->get_clock()->now();

    rclcpp::WallRate rate(rate_hz);
    while (rclcpp::ok()) {
        service.run_one();

        auto read_time = node->get_clock()->now();
        robot.read(read_time, read_time - last_read);
        last_read = read_time;

        auto update_time = node->get_clock()->now();
        cm.update(update_time, update_time - last_update);
        last_update = update_time;

        auto write_time = node->get_clock()->now();
        robot.write(write_time, write_time - last_write);
        last_write = write_time;

        rate.sleep();
        executor.spin_some();
    }

    return EXIT_SUCCESS;
}
