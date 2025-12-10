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
                pan_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                pan_inverted_ = joint.parameters.at("inverted") == "true";
                pan_offset_degrees_ = std::stod(joint.parameters.at("offset"));
            } else if (joint.name == "tilt_joint") {
                tilt_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                tilt_inverted_ = joint.parameters.at("inverted") == "true";
                tilt_offset_degrees_ = std::stod(joint.parameters.at("offset"));
            } else if (joint.name == "trigger_joint") {
                trigger_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                trigger_inverted_ = joint.parameters.at("inverted") == "true";
            }
        }

        for (auto & gpio : info.gpios) {
            if (gpio.name == "flywheel") {
                flywheel_in1_gpio_pin_ = std::stoi(gpio.parameters.at("in1_gpio_pin"));
                flywheel_in2_gpio_pin_ = std::stoi(gpio.parameters.at("in2_gpio_pin"));
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretRealHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        pan_angle_degrees_ = (pan_min_angle_degrees_ + pan_max_angle_degrees_) / 2.0;
        tilt_angle_degrees_ = tilt_min_angle_degrees_;
        trigger_distance_ = trigger_min_distance_;
        flywheel_enabled_ = 0.0;

        pca.set_pwm_freq(50);
        
        wiringPiSetupGpio();

        pinMode(flywheel_in1_gpio_pin_, OUTPUT);
        pinMode(flywheel_in2_gpio_pin_, OUTPUT);

        digitalWrite(flywheel_in1_gpio_pin_, LOW);
        digitalWrite(flywheel_in2_gpio_pin_, LOW);
        
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

        pca.set_pwm(pan_servo_id_, 0, angle_to_ticks(pan_angle_degrees_ - pan_offset_degrees_, pan_inverted_));
        pca.set_pwm(tilt_servo_id_, 0, angle_to_ticks(tilt_angle_degrees_ - tilt_offset_degrees_, tilt_inverted_));
        pca.set_pwm(trigger_servo_id_, 0, distance_to_ticks(trigger_distance_, trigger_inverted_));

        if (flywheel_enabled_ == 0.0) {
            digitalWrite(flywheel_in1_gpio_pin_, LOW);
            digitalWrite(flywheel_in2_gpio_pin_, LOW);
        } else {
            digitalWrite(flywheel_in1_gpio_pin_, HIGH);
            digitalWrite(flywheel_in2_gpio_pin_, LOW);
        }

        return hardware_interface::return_type::OK;
    }

    unsigned int TurretRealHardwareInterface::angle_to_ticks(double angle, bool inverted) {
        if (inverted) {
            angle = 180.0 - angle;
        }
        double microseconds = 500 + (angle / 180.0) * 2000.0; // 500us to 2500us
        double microseconds_per_tick = 20000.0 / 4096.0; // 20ms period, 4096 ticks
        return (unsigned int)(microseconds / microseconds_per_tick);
    }

    unsigned int TurretRealHardwareInterface::distance_to_ticks(double distance, bool inverted) {
        return angle_to_ticks(distance * 180.0, inverted);
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
