#include "dummy_robot_hardware/dummy_robot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dummy_robot_hardware
{
    hardware_interface::CallbackReturn DummyRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Get info parameters from URDF
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize internal variables
        robot_is_initialized_ = 0;
        emo_state_ = 0;
        emo_pressed_cmd_ = 0;

        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DummyRobotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),  joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DummyRobotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DummyRobotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DummyRobotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DummyRobotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Successfully initialized!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DummyRobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        state_interfaces.emplace_back(hardware_interface::StateInterface("emergency", "emo_state", &emo_state_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("robot_system", "initialized", &robot_is_initialized_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DummyRobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("emergency", "emo_pressed_cmd", &emo_pressed_cmd_));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DummyRobotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Activating ...please wait...");

        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        emo_state_ = 0;
        emo_pressed_cmd_ = 0;


        RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DummyRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Deactivating ...please wait...");

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(rclcpp::get_logger("DummyRobotSystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DummyRobotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Read from hardware and update internal variables.

        hw_velocities_[0] = 0.0;
        hw_positions_[0] = 0.0;

        hw_velocities_[1] = 0.0;
        hw_positions_[1] = 0.0;

        emo_state_ = emo_pressed_cmd_;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DummyRobotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Write to hardware from command variables.
        if(emo_state_)
        {
            robot_is_initialized_ = 0.0;

            hw_commands_[0] = -1.0;
            hw_commands_[1] = -1.0;
        }
        else
        {
            robot_is_initialized_ = 1.0;

            hw_commands_[0] = 0.0;
            hw_commands_[1] = 0.0;
        }

        // ...

        return hardware_interface::return_type::OK;
    }
} // namespace dummy_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dummy_robot_hardware::DummyRobotSystemHardware, hardware_interface::SystemInterface)

