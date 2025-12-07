#pragma once

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"

namespace turret_hardware {
    class TurretSimHardwareInterface : public hardware_interface::SystemInterface {
        public:
            TurretSimHardwareInterface() = default;

            // -------------------------------
            // LifecycleNodeInterface overrides
            // -------------------------------

            hardware_interface::CallbackReturn on_init([[maybe_unused]] const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;

            // --------------------------------
            // SystemInterface overrides
            // --------------------------------

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;
            hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;

        private:
            double pan_angle_;
            double tilt_angle_;
            double trigger_distance_;
            double flywheel_enabled_;
    };
}
