#include "robot_firmware/diffbot_system.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
namespace robot_firmware
{
DiffBotSystemHardware::DiffBotSystemHardware() : node_(std::make_shared<rclcpp::Node>("diffbot_interface_node"))

{
  feedback_subscription_ = node_->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/motor/feedback", 10,
    [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
      this->processFeedback(msg);
    });
  
  velocity_subscription_ = node_->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/motor/feedback_vel", 10,
    [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
      this->processVelFeedback(msg);
    });  

  cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/motor/cmd", 10);
}

DiffBotSystemHardware::~DiffBotSystemHardware() {}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffBotSystemHardware::processFeedback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 2) {
    hw_positions_[0] = ((msg->data[0] * 2.0 * M_PI) / 4096.0) * 0.033;  // left_wheel_joint position cumilative
    hw_positions_[1] = ((msg->data[1] * 2.0 * M_PI) / 4096.0) * 0.033;  // right_wheel_joint position cumilative
  }
}

void DiffBotSystemHardware::processVelFeedback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 2) {
    hw_velocities_[1] = ((msg->data[0] * 2.0 * M_PI) / 60.0)*0.033;  // left_base_joint speed
    hw_velocities_[0] = ((msg->data[1] * 2.0 * M_PI) / 60.0)*0.033;  // right_base_joint speed
  }
}



hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto cmd_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
  cmd_msg->data.push_back(hw_commands_[0]);  // left motor command
  cmd_msg->data.push_back(hw_commands_[1]);  // right motor command
  cmd_publisher_->publish(*cmd_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace robot_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_firmware::DiffBotSystemHardware, hardware_interface::SystemInterface)
