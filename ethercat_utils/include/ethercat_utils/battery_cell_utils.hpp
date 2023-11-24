#ifndef BATTERY_CELL_UTILS_HPP_
#define BATTERY_CELL_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ethercat_utils_msgs/msg/gpio_array.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <chrono>


namespace ethercat_utils {

class BatteryCellUtils : public rclcpp::Node
{
  public:
    BatteryCellUtils(const std::string name,
                     const rclcpp::NodeOptions option = (rclcpp::NodeOptions()
                                                         .allow_undeclared_parameters(true)
                                                         .automatically_declare_parameters_from_overrides(true)
                                                         )
                     );
    ~BatteryCellUtils();

  private:
    bool new_lubrication_msg_, lubrication_state_, new_ati_sensor_msg_;
    std::mutex lubrication_mtx_, ati_sensor_mtx_;
    std::string lubrication_command_controller_name_,
                lubrication_state_controller_name_,
                lubrication_command_interface_name_,
                lubrication_state_interface_name_,
                pneumatic_gripper_controller_name_,
                pneumatic_gripper_command_interface_name_,
                ati_sensor_controller_name_,
                ati_sensor_command_interface_name_;

    std::vector<double> ati_sensor_values_;
    std::vector<std::string> ati_sensor_state_interface_names_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ati_sensor_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lubrication_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pneumatic_gripper_server_;
    rclcpp::Publisher<ethercat_utils_msgs::msg::GPIOArray>::SharedPtr lubrication_publisher_,
                                                                      pneumatic_gripper_publisher_,
                                                                      ati_sensor_publisher_;

    rclcpp::Subscription<control_msgs::msg::InterfaceValue>::SharedPtr lubrication_subscription_,
                                                                       ati_sensor_subscription_;

    void read_lubrication_state_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg);
    void read_ati_sensor_values_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg);
    void lubrication_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void pneumatic_gripper_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void reset_ati_sensor_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
};

}// namespace ethercat_utils

#endif  // BATTERY_CELL_UTILS_HPP_
