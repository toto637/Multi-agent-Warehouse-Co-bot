#ifndef DIFFBOT_SYSTEM_HPP
#define DIFFBOT_SYSTEM_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include <vector>
#include <string>
#include "std_msgs/msg/int64_multi_array.hpp"
#include "rclcpp/subscription.hpp"

namespace robot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  DiffBotSystemHardware();
  virtual ~DiffBotSystemHardware();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr feedback_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr velocity_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_publisher_;
  
  void processFeedback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
  void processVelFeedback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);

  double hw_start_sec_;
  double hw_stop_sec_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

};  // namespace autonav_firmware
}
#endif  // DIFFBOT_SYSTEM_HPP
