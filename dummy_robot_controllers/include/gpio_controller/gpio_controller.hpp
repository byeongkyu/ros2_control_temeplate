#ifndef DUMMY_ROBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define DUMMY_ROBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"

namespace dummy_robot_controllers
{
    enum CommandInterfaces
    {
        EMO_PRESSED_CMD = 0u,
    };

    enum StateInterfaces
    {
        EMO_STATE = 0u,
    };

    class GPIOController : public controller_interface::ControllerInterface
    {
        public:
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_init() override;

        private:
            void callback_emo_pressed(const std_msgs::msg::Bool & msg);

        private:
            std_msgs::msg::Bool emo_state_;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub_emo_state_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emo_pressed_;

            static constexpr double ASYNC_WAITING = 2.0;
    };
}
#endif //DUMMY_ROBOT_CONTROLLERS__GPIO_CONTROLLER_HPP_