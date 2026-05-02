#include "megajaw_hardware/MotorDriver.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace megajaw_hardware {

class MegaJawHardware : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        for (size_t i = 0; i < info_.joints.size(); i++) {
            if (info_.joints[i].name == "left_wheel_base_joint") {
                left_idx_ = i;
            } else if (info_.joints[i].name == "right_wheel_base_joint") {
                right_idx_ = i;
            }
        }

        // 3. Parse Pins
        int lpA = std::stoi(info_.hardware_parameters.at("left_wheel_pin_a"));
        int lpB = std::stoi(info_.hardware_parameters.at("left_wheel_pin_b"));
        int rpA = std::stoi(info_.hardware_parameters.at("right_wheel_pin_a"));
        int rpB = std::stoi(info_.hardware_parameters.at("right_wheel_pin_b"));

        driver_ = std::make_unique<MotorDriver>(rpA, rpB, lpA, lpB);

        hw_commands_.resize(info_.joints.size(), 0.0);
        hw_states_pos_.resize(info_.joints.size(), 0.0);
        hw_states_vel_.resize(info_.joints.size(), 0.0);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(info_.joints[i].name, "position", &hw_states_pos_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_states_vel_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_commands_[i]);
        }
        return command_interfaces;
    }

    // REQUIRED: Even if it's just dummy data for now
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override {
        for (size_t i = 0; i < hw_states_pos_.size(); i++) {
            hw_states_vel_[i] = hw_commands_[i];
            hw_states_pos_[i] += hw_states_vel_[i] * period.seconds();
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
        float left_pct = static_cast<float>(hw_commands_[left_idx_]) / 100.0;
        float right_pct = static_cast<float>(hw_commands_[right_idx_]) / 100.0;

        driver_->setLeftMotor(left_pct);
        driver_->setRightMotor(right_pct);

	// RCLCPP_INFO(rclcpp::get_logger("MegaJawHardware"),
        //    "write() called: left_vel=%.2f, right_vel=%.2f, left_pct=%.2f, right_pct=%.2f",
        //    hw_commands_[left_idx_], hw_commands_[right_idx_], left_pct, right_pct);

        return hardware_interface::return_type::OK;
    }

private:
    std::unique_ptr<MotorDriver> driver_;
    size_t left_idx_ = 0, right_idx_ = 1; 
    std::vector<double> hw_commands_, hw_states_pos_, hw_states_vel_;
};

}

PLUGINLIB_EXPORT_CLASS(megajaw_hardware::MegaJawHardware, hardware_interface::SystemInterface)

