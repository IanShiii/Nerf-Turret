#include "turret_hardware/turret_sim_hardware.hpp"

namespace turret_hardware {

    // -------------------------------
    // LifecycleNodeInterface overrides
    // -------------------------------

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_init([[maybe_unused]]const hardware_interface::HardwareInfo & info) {
        pan_angle_ = 0.0;
        tilt_angle_ = 0.0;
        trigger_distance_ = 0.0;
        flywheel_enabled_ = 0.0;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // --------------------------------
    // SystemInterface overrides
    // --------------------------------

    std::vector<hardware_interface::StateInterface> TurretSimHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TurretSimHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("pan_joint", "angle", &pan_angle_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("tilt_joint", "angle", &tilt_angle_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("trigger_joint", "distance", &trigger_distance_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("flywheel", "enabled", &flywheel_enabled_));
        return command_interfaces;
    }

    hardware_interface::return_type TurretSimHardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurretSimHardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(turret_hardware::TurretSimHardwareInterface, hardware_interface::SystemInterface)
