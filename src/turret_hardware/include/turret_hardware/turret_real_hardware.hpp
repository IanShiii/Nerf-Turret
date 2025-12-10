#pragma once

#include <wiringPi.h>

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#include <PiPCA9685/PCA9685.h>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"

namespace turret_hardware {
    class TurretRealHardwareInterface : public hardware_interface::SystemInterface {
        public:
            TurretRealHardwareInterface() = default;

            // -------------------------------
            // LifecycleNodeInterface overrides
            // -------------------------------

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
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
            PiPCA9685::PCA9685 pca;

            // --------------------------------
            // PCA9685 servo numbers and GPIO Pins
            // --------------------------------

            int pan_servo_id_;
            int tilt_servo_id_;
            int trigger_servo_id_;
            int flywheel_in1_gpio_pin_;
            int flywheel_in2_gpio_pin_;

            // --------------------------------
            // Software Limits
            // --------------------------------

            double pan_min_angle_degrees_;
            double pan_max_angle_degrees_;
            double tilt_min_angle_degrees_;
            double tilt_max_angle_degrees_;
            double trigger_min_distance_;
            double trigger_max_distance_;

            // --------------------------------
            // Inversions
            // --------------------------------

            bool pan_inverted_;
            bool tilt_inverted_;
            bool trigger_inverted_;

            // --------------------------------
            // Inversions
            // --------------------------------

            double pan_offset_degrees_;
            double tilt_offset_degrees_;

            // --------------------------------
            // Targets
            // --------------------------------

            double pan_angle_degrees_;
            double tilt_angle_degrees_;
            double trigger_distance_;
            double flywheel_enabled_;

            /**
             * @brief Converts an angle in degrees to ticks for the PCA9685.
             * @param angle Angle in degrees [0, 180]
             */
            unsigned int angle_to_ticks(double angle, bool inverted);

            /**
             * @brief Converts a distance for the trigger to ticks for the PCA9685.
             * @param distance [0, 1]
             */
            unsigned int distance_to_ticks(double distance, bool inverted);

            /**
             * @brief Clamps command values to their respective min/max ranges.
             */
            void clamp_command_values();
    };
}
