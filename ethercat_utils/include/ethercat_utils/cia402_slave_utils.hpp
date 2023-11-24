#ifndef CIA402_SLAVE_UTILS_HPP_
#define CIA402_SLAVE_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ethercat_utils_msgs/msg/gpio_array.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <mutex>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <chrono>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <limits.h>

enum DeviceState
{
  STATE_UNDEFINED = 0,
  STATE_START = 1,
  STATE_NOT_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON_DISABLED,
  STATE_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON,
  STATE_OPERATION_ENABLED,
  STATE_QUICK_STOP_ACTIVE,
  STATE_FAULT_REACTION_ACTIVE,
  STATE_FAULT
};

enum ModeOfOperation
{
  MODE_NO_MODE                = 0,
  MODE_PROFILED_POSITION      = 1,
  MODE_PROFILED_VELOCITY      = 3,
  MODE_PROFILED_TORQUE        = 4,
  MODE_HOMING                 = 6,
  MODE_INTERPOLATED_POSITION  = 7,
  MODE_CYCLIC_SYNC_POSITION   = 8,
  MODE_CYCLIC_SYNC_VELOCITY   = 9,
  MODE_CYCLIC_SYNC_TORQUE     = 10
};

const std::map<DeviceState, std::string> DEVICE_STATE_STR = {
  {STATE_START, "Start"},
  {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
  {STATE_SWITCH_ON_DISABLED, "Switch on Disabled"},
  {STATE_READY_TO_SWITCH_ON, "Ready to Switch On"},
  {STATE_SWITCH_ON, "Switch On"},
  {STATE_OPERATION_ENABLED, "Operation Enabled"},
  {STATE_QUICK_STOP_ACTIVE, "Quick Stop Active"},
  {STATE_FAULT_REACTION_ACTIVE, "Fault Reaction Active"},
  {STATE_FAULT, "Fault"},
  {STATE_UNDEFINED, "Undefined State"}
};

namespace ethercat_utils {

class Cia402SlaveUtils : public rclcpp::Node
{
  public:
    Cia402SlaveUtils(const std::string name,
                     const rclcpp::NodeOptions option = (rclcpp::NodeOptions()
                                                         .allow_undeclared_parameters(true)
                                                         .automatically_declare_parameters_from_overrides(true)
                                    )
                     );
    ~Cia402SlaveUtils();

  private:
    std::mutex moo_mtx_, status_word_mtx_;
    std::string moo_controller_name_,
                joint_trajectory_controller_name_,
                control_word_controller_name_,
                control_word_command_interface_name_,
                status_word_controller_name_,
                status_word_state_interface_name_,
                mode_of_operation_controller_name_,
                mode_of_operation_command_interface_name_,
                mode_of_operation_display_controller_name_,
                mode_of_operation_display_state_interface_name_;

    int current_status_word_;
    bool new_moo_msg_, homing_executed_ = false;
    DeviceState current_status_;
    ModeOfOperation current_moo_;
    ModeOfOperation current_moo_command_ = ModeOfOperation::MODE_HOMING;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr perform_homing_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_position_control_server_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr activate_jt_controller_client_;

    rclcpp::Publisher<ethercat_utils_msgs::msg::GPIOArray>::SharedPtr moo_publisher_;
    rclcpp::Publisher<ethercat_utils_msgs::msg::GPIOArray>::SharedPtr ctrl_word_publisher_;

    rclcpp::Subscription<control_msgs::msg::InterfaceValue>::SharedPtr moo_subscription_;
    rclcpp::Subscription<control_msgs::msg::InterfaceValue>::SharedPtr status_word_subscription_;

    void reset_fault_cb(             const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void perform_homing_cb(          const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void start_position_control_cb(  const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void read_moo_cb(   const control_msgs::msg::InterfaceValue::SharedPtr msg);
    void read_status_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg);

    DeviceState deviceState(uint16_t status_word);

    bool send_and_wait_for_control_word(double ctrl_word);
    bool change_mode_of_operation(int moo);
};

} // namespace ethercat_utils

#endif  // CIA402_SLAVE_UTILS_HPP_
