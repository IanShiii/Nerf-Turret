#include "turret_hardware/turret_hardware.hpp"

namespace turret_hardware {

    // -------------------------------
    // LifecycleNodeInterface overrides
    // -------------------------------

    hardware_interface::CallbackReturn TurretHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        for (auto & joint : info.joints) {
            if (joint.name == "pan_joint") {
                pan_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
            } else if (joint.name == "tilt_joint") {
                tilt_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
            } else if (joint.name == "trigger_joint") {
                trigger_servo_gpio_pin_ = std::stoi(joint.parameters.at("gpio_pin"));
            } else if (joint.name == "flywheel_joint") {
                flywheel_in1_gpio_pin_ = std::stoi(joint.parameters.at("in1_gpio_pin"));
                flywheel_in2_gpio_pin_ = std::stoi(joint.parameters.at("in2_gpio_pin"));
            }
        }

        pan_angle_ = 0.0;
        tilt_angle_ = 0.0;
        trigger_distance_ = 0.0;
        flywheel_speed_ = 0.0;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        wiringPiSetupGpio();

        pwmSetMode(PWM_MODE_MS);
        pwmSetClock(192);
        pwmSetRange(2000);

        pinMode(pan_servo_gpio_pin_, PWM_OUTPUT);
        pinMode(tilt_servo_gpio_pin_, PWM_OUTPUT);
        pinMode(trigger_servo_gpio_pin_, PWM_OUTPUT);
        pinMode(flywheel_enable_gpio_pin_, OUTPUT);

        pinMode(flywheel_in1_gpio_pin_, OUTPUT);
        pinMode(flywheel_in2_gpio_pin_, OUTPUT);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        // TODO: Set all outputs to safe state
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // --------------------------------
    // SystemInterface overrides
    // --------------------------------

    std::vector<hardware_interface::StateInterface> TurretHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TurretHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("pan_joint", "angle", &pan_angle_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("tilt_joint", "angle", &tilt_angle_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("trigger_joint", "distance", &trigger_distance_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("flywheel", "speed", &flywheel_speed_));
        return command_interfaces;
    }

    hardware_interface::return_type TurretHardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurretHardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        pwmWrite(pan_servo_gpio_pin_, angle_to_pwm(pan_angle_));
        pwmWrite(tilt_servo_gpio_pin_, angle_to_pwm(tilt_angle_));
        pwmWrite(trigger_servo_gpio_pin_, distance_to_pwm(trigger_distance_));

        digitalWrite(flywheel_in1_gpio_pin_, 1);
        digitalWrite(flywheel_in2_gpio_pin_, 0);

        return hardware_interface::return_type::OK;
    }

    unsigned int TurretHardwareInterface::angle_to_pwm(double angle) {
        return (unsigned int)(100 + (angle / 180.0) * 100);
    }

    unsigned int TurretHardwareInterface::distance_to_pwm(double distance) {
        return (unsigned int)(100 + distance * 100);
    }
}

PLUGINLIB_EXPORT_CLASS(turret_hardware::TurretHardwareInterface, hardware_interface::SystemInterface)
