#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "megajaw_hardware/GripperDriver.hpp"

class GripperControlNode : public rclcpp::Node
{
public:
    GripperControlNode()
        : Node("gripper_control_node")
    {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
        this->declare_parameter<int>("baudrate", 115200);

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Initialize gripper driver
        gripper_driver_ = std::make_unique<GripperDriver>(serial_port, baudrate);

        // Subscribe to /cmd_grip
        grip_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd_grip", 10,
            std::bind(&GripperControlNode::gripCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gripper control node started, listening on /cmd_grip");
    }

private:
    void gripCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "OPEN")
        {
            RCLCPP_INFO(this->get_logger(), "Received OPEN command");
            gripper_driver_->openGripper();
        }
        else if (msg->data == "CLOSED")
        {
            RCLCPP_INFO(this->get_logger(), "Received CLOSED command");
            gripper_driver_->closeGripper();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown gripper command: '%s' (expected 'OPEN' or 'CLOSED')", msg->data.c_str());
        }
    }

    std::unique_ptr<GripperDriver> gripper_driver_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grip_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperControlNode>());
    rclcpp::shutdown();
    return 0;
}
