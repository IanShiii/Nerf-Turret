#include "turret_hardware/turret_real_hardware.hpp"

namespace turret_hardware {

    // -------------------------------
    // LifecycleNodeInterface overrides
    // -------------------------------

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        pan_min_angle_degrees_ = info.limits.at("pan_joint").min_position;
        pan_max_angle_degrees_ = info.limits.at("pan_joint").max_position;
        tilt_min_angle_degrees_ = info.limits.at("tilt_joint").min_position;
        tilt_max_angle_degrees_ = info.limits.at("tilt_joint").max_position;
        trigger_min_distance_ = info.limits.at("trigger_joint").min_position;
        trigger_max_distance_ = info.limits.at("trigger_joint").max_position;
        
        for (auto & joint : info.joints) {
            if (joint.name == "pan_joint") {
                pan_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
                pan_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "tilt_joint") {
                tilt_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
                tilt_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "trigger_joint") {
                trigger_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
                trigger_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "flywheel_joint") {
                flywheel_in1_gpio_pin_ = std::stoi(joint.parameters.at("in1_gpio_pin"));
                flywheel_in2_gpio_pin_ = std::stoi(joint.parameters.at("in2_gpio_pin"));
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        pan_angle_degrees_ = (pan_min_angle_degrees_ + pan_max_angle_degrees_) / 2.0;
        tilt_angle_degrees_ = (tilt_min_angle_degrees_ + tilt_max_angle_degrees_) / 2.0;
        trigger_distance_ = trigger_min_distance_;
        flywheel_enabled_ = 0.0;
        
        wiringPiSetupGpio();

        pinMode(pan_servo_gpio_pin_, PWM_OUTPUT);
        pinMode(tilt_servo_gpio_pin_, PWM_OUTPUT);
        pinMode(trigger_servo_gpio_pin_, PWM_OUTPUT);

        pinMode(flywheel_in1_gpio_pin_, OUTPUT);
        pinMode(flywheel_in2_gpio_pin_, OUTPUT);

        pwmSetMode(PWM_MODE_MS);
        pwmSetClock(192);
        pwmSetRange(2000);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        // TODO: Set all outputs to safe state
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // --------------------------------
    // SystemInterface overrides
    // --------------------------------

    std::vector<hardware_interface::StateInterface> TurretRealHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TurretRealHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("pan_joint", "angle", &pan_angle_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("tilt_joint", "angle", &tilt_angle_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("trigger_joint", "distance", &trigger_distance_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("flywheel", "enabled", &flywheel_enabled_));
        return command_interfaces;
    }

    hardware_interface::return_type TurretRealHardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurretRealHardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        clamp_command_values();

        pwmWrite(pan_servo_gpio_pin_, angle_to_pwm(pan_angle_degrees_, pan_inverted_));
        pwmWrite(tilt_servo_gpio_pin_, angle_to_pwm(tilt_angle_degrees_, tilt_inverted_));
        pwmWrite(trigger_servo_gpio_pin_, distance_to_pwm(trigger_distance_, trigger_inverted_));

        if (flywheel_enabled_ == 0.0) {
            digitalWrite(flywheel_in1_gpio_pin_, 0);
            digitalWrite(flywheel_in2_gpio_pin_, 0);
        } else {
            digitalWrite(flywheel_in1_gpio_pin_, 1);
            digitalWrite(flywheel_in2_gpio_pin_, 0);
        }

        return hardware_interface::return_type::OK;
    }

    unsigned int TurretRealHardwareInterface::angle_to_pwm(double angle, bool inverted) {
        if (inverted) {
            angle = 180.0 - angle;
        }
        return (unsigned int)(50 + (angle / 180.0) * 200);
    }

    unsigned int TurretRealHardwareInterface::distance_to_pwm(double distance, bool inverted) {
        if (inverted) {
            distance = 1.0 - distance;
        }
        return (unsigned int)(50 + distance * 200);
    }

    void TurretRealHardwareInterface::clamp_command_values() {
        if (pan_angle_degrees_ < pan_min_angle_degrees_) {
            pan_angle_degrees_ = pan_min_angle_degrees_;
        } else if (pan_angle_degrees_ > pan_max_angle_degrees_) {
            pan_angle_degrees_ = pan_max_angle_degrees_;
        }

        if (tilt_angle_degrees_ < tilt_min_angle_degrees_) {
            tilt_angle_degrees_ = tilt_min_angle_degrees_;
        } else if (tilt_angle_degrees_ > tilt_max_angle_degrees_) {
            tilt_angle_degrees_ = tilt_max_angle_degrees_;
        }

        if (trigger_distance_ < trigger_min_distance_) {
            trigger_distance_ = trigger_min_distance_;
        } else if (trigger_distance_ > trigger_max_distance_) {
            trigger_distance_ = trigger_max_distance_;
        }
    }
}

PLUGINLIB_EXPORT_CLASS(turret_hardware::TurretRealHardwareInterface, hardware_interface::SystemInterface)
