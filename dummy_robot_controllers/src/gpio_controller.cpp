#include "gpio_controller/gpio_controller.hpp"
#include <string>

namespace dummy_robot_controllers
{
    controller_interface::CallbackReturn GPIOController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init...");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("emergency/emo_pressed_cmd");


        RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration...");
        return config;
    }

    controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("emergency/emo_state");
        config.names.emplace_back("robot_system/initialized");

        RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration...");
        return config;
    }

    controller_interface::return_type GPIOController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // publish emo_state
        auto current_emo_state = static_cast<bool>(state_interfaces_[StateInterfaces::EMO_STATE].get_value());
        auto current_initialized = static_cast<bool>(state_interfaces_[StateInterfaces::ROBOT_INITIALIZED].get_value());

        if(current_emo_state != emo_state_.data)
        {
            emo_state_.data = current_emo_state;
        }
        robot_initialized_.data = current_initialized;

        pub_emo_state_->publish(emo_state_);
        pub_robot_init_state_->publish(robot_initialized_);
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_configure...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0)
        // {
        //     RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
        //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // }

        try
        {
            pub_emo_state_ = get_node()->create_publisher<std_msgs::msg::Bool>("~/emo_state", rclcpp::SystemDefaultsQoS());
            pub_robot_init_state_ = get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_initialized", rclcpp::SystemDefaultsQoS());
            sub_emo_pressed_ = get_node()->create_subscription<std_msgs::msg::Bool>(
                "~/emo_pressed", 10, std::bind(&GPIOController::callback_emo_pressed, this, std::placeholders::_1));
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "on_activate...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        try
        {
            pub_emo_state_.reset();
            pub_robot_init_state_.reset();
            sub_emo_pressed_.reset();
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void GPIOController::callback_emo_pressed(const std_msgs::msg::Bool & msg)
    {
        command_interfaces_[CommandInterfaces::EMO_PRESSED_CMD].set_value(msg.data ? 1.0 : 0.0);
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dummy_robot_controllers::GPIOController, controller_interface::ControllerInterface)