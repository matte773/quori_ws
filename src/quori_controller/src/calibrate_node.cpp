#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_value.hpp>

#include "quori_controller/CalibrateConfig.hpp" // Make sure this is updated to ROS2 style

class CalibrateNode : public rclcpp::Node
{
public:
    CalibrateNode()
    : Node("calibrate_node")
    {
        this->declare_parameter("base_turret_offset", 0.0);

        // Setup a callback to handle changes to parameters.
        param_sub_ = this->add_on_set_parameters_callback(
            std::bind(&CalibrateNode::parametersCallback, this, std::placeholders::_1));
    }

private:
    // Handle dynamic parameter changes
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "base_turret_offset")
            {
                auto new_offset = parameter.as_double();
                // Handle the new parameter value as needed
                RCLCPP_INFO(this->get_logger(), "Updated base_turret_offset: %f", new_offset);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Parameter '%s' not recognized.", parameter.get_name().c_str());
                result.successful = false;
                result.reason = "Parameter not recognized";
            }
        }

        return result;
    }

    // Member variable to hold the parameters subscription.
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
