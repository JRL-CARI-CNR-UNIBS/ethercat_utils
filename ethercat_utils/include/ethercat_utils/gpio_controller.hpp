#ifndef ETHERCAT_UTILS__GPIO_CONTROLLER_HPP_
#define ETHERCAT_UTILS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "controller_interface/controller_interface.hpp"
#include <ethercat_utils_msgs/msg/gpio_array.hpp>

namespace ethercat_utils
{
using CmdType = ethercat_utils_msgs::msg::GPIOArray;

class GPIOController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GPIOController)

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

private:
  std::vector<std::string> inputs_;
  std::vector<std::string> outputs_;

protected:
  void initMsgs();

  // internal commands
  std::shared_ptr<CmdType> output_cmd_ptr_;

  // publisher
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::InterfaceValue>> gpio_publisher_;
  control_msgs::msg::InterfaceValue gpio_msg_;

  // subscriber
  rclcpp::Subscription<CmdType>::SharedPtr subscription_command_;
};
}  // namespace ros2_control_demo_example_10

#endif  // ETHERCAT_UTILS__GPIO_CONTROLLER_HPP_
