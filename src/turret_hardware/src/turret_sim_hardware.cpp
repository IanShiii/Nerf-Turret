#include "turret_hardware/turret_sim_hardware.hpp"

namespace turret_hardware {

    // -------------------------------
    // LifecycleNodeInterface overrides
    // -------------------------------

    hardware_interface::CallbackReturn TurretSimHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        pan_min_angle_degrees_ = info.limits.at("pan_joint").min_position;
        pan_max_angle_degrees_ = info.limits.at("pan_joint").max_position;
        tilt_min_angle_degrees_ = info.limits.at("tilt_joint").min_position;
        tilt_max_angle_degrees_ = info.limits.at("tilt_joint").max_position;
        trigger_min_distance_ = info.limits.at("trigger_joint").min_position;
        trigger_max_distance_ = info.limits.at("trigger_joint").max_position;

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
        command_interfaces.emplace_back(hardware_interface::CommandInterface("pan_joint", "angle", &pan_angle_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("tilt_joint", "angle", &tilt_angle_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("trigger_joint", "distance", &trigger_distance_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("flywheel", "enabled", &flywheel_enabled_));
        return command_interfaces;
    }

    hardware_interface::return_type TurretSimHardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurretSimHardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        clamp_command_values();

        RCLCPP_INFO(
            get_logger(),
            "Writing to turret hardware: pan_angle=%.2f, tilt_angle=%.2f, trigger_distance=%.2f, flywheel_enabled=%.2f",
            pan_angle_degrees_,
            tilt_angle_degrees_,
            trigger_distance_,
            flywheel_enabled_
        );

        return hardware_interface::return_type::OK;
    }

    void TurretSimHardwareInterface::clamp_command_values() {
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

PLUGINLIB_EXPORT_CLASS(turret_hardware::TurretSimHardwareInterface, hardware_interface::SystemInterface)
