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
                flywheel_enable_gpio_pin_ = std::stoi(joint.parameters.at("enable_gpio_pin"));
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        gpioInitialise();

        gpioSetMode(pan_servo_gpio_pin_, PI_OUTPUT);
        gpioSetMode(tilt_servo_gpio_pin_, PI_OUTPUT);
        gpioSetMode(trigger_servo_gpio_pin_, PI_OUTPUT);
        gpioSetMode(flywheel_in1_gpio_pin_, PI_OUTPUT);
        gpioSetMode(flywheel_in2_gpio_pin_, PI_OUTPUT);
        gpioSetMode(flywheel_enable_gpio_pin_, PI_OUTPUT);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretHardwareInterface::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        gpioTerminate();
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
        gpioServo(pan_servo_gpio_pin_, angle_to_pwm(pan_angle_));
        gpioServo(tilt_servo_gpio_pin_, angle_to_pwm(tilt_angle_));
        gpioServo(trigger_servo_gpio_pin_, distance_to_pwm(trigger_distance_));

        gpioWrite(flywheel_in1_gpio_pin_, 1);
        gpioWrite(flywheel_in2_gpio_pin_, 0);
        gpioPWM(flywheel_enable_gpio_pin_, static_cast<unsigned int>(flywheel_speed_));

        return hardware_interface::return_type::OK;
    }

    unsigned int TurretHardwareInterface::angle_to_pwm(double angle) {
        return static_cast<unsigned int>(500.0 + (angle / 180.0) * 2000.0);
    }

    unsigned int TurretHardwareInterface::distance_to_pwm(double distance) {
        return static_cast<unsigned int>(500.0 + distance * 2000.0);
    }
}

PLUGINLIB_EXPORT_CLASS(turret_hardware::TurretHardwareInterface, hardware_interface::SystemInterface)
