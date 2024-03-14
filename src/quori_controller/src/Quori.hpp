#pragma once

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

#include "SerialDevice.hpp"
#include "Joint.hpp"

#include "message.hpp"

#include <map>
#include <unordered_map>
#include <vector>
#include <optional>

namespace quori_controller
{
  class Quori : public hardware_interface::SystemInterface
  {
  public:
    // Callback to be implemented by the user, for initializing the hardware
    // This should include hardware resource initialization, first read and state preparation
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    // Read the state from the hardware
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // Write the command to the hardware
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // The read function should be implemented by the user and should perform a read operation
    // on the hardware and populate the member variables with the read values
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // Write the received commands to the hardware
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  private:
    void on_base_vel_status_(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void on_base_turret_pos_(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr base_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr base_holo_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr base_offset_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr base_vel_status_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr base_turret_pos_;

    std::vector<SerialDevice::Ptr> devices_;
    std::map<SerialDevice::Ptr, quori::message::States> device_states_;

    // Interfaces replaced by export_state_interfaces and export_command_interfaces
    std::map<SerialDevice::Ptr, std::vector<std::size_t>> device_joints_;
    std::size_t max_device_joints_;
    double *device_joint_buffer_;

    float base_offset_;

    std::unordered_map<std::string, std::size_t> joint_indices_;
    std::vector<Joint::Ptr> joints_;

    Joint::Ptr base_turret_;
    Joint::Ptr base_left_;
    Joint::Ptr base_right_;

    Joint::Ptr base_x_;
    Joint::Ptr base_y_;
    Joint::Ptr base_angle_;
    Joint::Ptr base_mode_;

    std::optional<geometry_msgs::msg::Vector3::SharedPtr> base_vel_;

    rclcpp::Time last_read_;
  };
}
