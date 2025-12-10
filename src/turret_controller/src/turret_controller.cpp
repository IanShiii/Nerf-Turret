#include "turret_controller/turret_controller.hpp"

namespace turret_controller {
    controller_interface::CallbackReturn TurretController::on_init() {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration TurretController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "pan_joint/angle",
            "tilt_joint/angle",
            "trigger_joint/distance",
            "flywheel/enabled"
        };
        return config;
    }

    controller_interface::InterfaceConfiguration TurretController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::CallbackReturn TurretController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TurretController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        pan_angle_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            "/turret_controller/target_pan_angle",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                pan_angle_ = msg->data;
            }
        );

        tilt_angle_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            "/turret_controller/target_tilt_angle",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                tilt_angle_ = msg->data;
            }
        );

        trigger_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "/turret_controller/trigger",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
                (void)request;
                if (trigger_requested_.load()) {
                    response->success = false;
                    response->message = "Turret is already firing.";
                    return;
                }

                trigger_requested_.store(true);
                std::thread(&TurretController::trigger_shoot, this).detach();
                response->success = true;
            }
        );

        flywheel_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
            "/turret_controller/set_flywheel",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
                flywheel_enabled_ = request->data;
                response->success = true;
            }
        );

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TurretController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        pan_angle_sub_.reset();
        tilt_angle_sub_.reset();
        trigger_service_.reset();
        flywheel_service_.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type TurretController::update([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        for (auto & command_interface : command_interfaces_) {
            if (command_interface.get_name() == "pan_joint/angle") {
                (void)command_interface.set_value(pan_angle_);
            } else if (command_interface.get_name() == "tilt_joint/angle") {
                (void)command_interface.set_value(tilt_angle_);
            } else if (command_interface.get_name() == "trigger_joint/distance") {
                (void)command_interface.set_value(trigger_distance_.load());
            } else if (command_interface.get_name() == "flywheel/enabled") {
                (void)command_interface.set_value(flywheel_enabled_ ? 1.0 : 0.0);
            }
        }

        return controller_interface::return_type::OK;
    }

    void TurretController::trigger_shoot() {
        trigger_distance_.store(1.0); // Extend the trigger
        rclcpp::sleep_for(std::chrono::milliseconds(400));
        trigger_distance_.store(0.0); // Retract the trigger
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        trigger_requested_.store(false);
    }
}

PLUGINLIB_EXPORT_CLASS(turret_controller::TurretController, controller_interface::ControllerInterface)
