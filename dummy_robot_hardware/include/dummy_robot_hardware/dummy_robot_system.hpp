#ifndef DUMMY_ROBOT_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_
#define DUMMY_ROBOT_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dummy_robot_hardware/visibility_control.h"

namespace dummy_robot_hardware
{
class DummyRobotSystemHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DummyRobotSystemHardware)

        DUMMY_ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        DUMMY_ROBOT_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        double emo_state_;
        double emo_pressed_cmd_;
};

}  // namespace dummy_robot_hardware

#endif  // DUMMY_ROBOT_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_