#pragma once

#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <atomic>

namespace turret_controller {
    class TurretController : public controller_interface::ControllerInterface {
        public:
            TurretController() = default;

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::return_type update([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        private:
            double pan_angle_{0.0};
            double tilt_angle_{0.0};
            bool flywheel_enabled_{false};
            std::atomic<double> trigger_distance_{0.0};
            std::atomic<bool> trigger_requested_{false};

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_angle_sub_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tilt_angle_sub_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr flywheel_service_;

            /**
             * @brief Extends the trigger, waits a short time, then retracts the trigger.
             */
            void trigger_shoot();
    };
}
