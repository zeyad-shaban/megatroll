#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
        grip_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands", 10,
            std::bind(&GripperControlNode::gripCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gripper control node started, listening on /cmd_grip");
    }

private:
    void gripCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2 && msg->data[0] != 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "Received OPEN command via Float64MultiArray");
            gripper_driver_->openGripper();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received CLOSED command via Float64MultiArray");
            gripper_driver_->closeGripper();
        }
    }

    std::unique_ptr<GripperDriver> gripper_driver_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grip_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperControlNode>());
    rclcpp::shutdown();
    return 0;
}
