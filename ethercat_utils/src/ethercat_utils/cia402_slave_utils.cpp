#include <ethercat_utils/cia402_slave_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace ethercat_utils {

Cia402SlaveUtils::Cia402SlaveUtils(const std::string name,
                                   const rclcpp::NodeOptions option)
    : rclcpp::Node(name, option) {

  if (this->get_parameter("joint_trajectory_controller_name", joint_trajectory_controller_name_))
  {
    if (joint_trajectory_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No joint_trajectory_controller_name parameter");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "joint_trajectory_controller_name: " << joint_trajectory_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No joint_trajectory_controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("control_word.controller_name", control_word_controller_name_))
  {
    if (control_word_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No control_word.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "control_word.controller_name: " << control_word_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No control_word.controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("control_word.command_interface", control_word_command_interface_name_))
  {
    if (control_word_command_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No control_word.command_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "control_word.command_interface: " << control_word_command_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No control_word.command_interface parameter");
    exit(1);
  }
  if (this->get_parameter("status_word.controller_name", status_word_controller_name_))
  {
    if (status_word_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No status_word.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "status_word.controller_name: " << status_word_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No status_word.controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("status_word.state_interface", status_word_state_interface_name_))
  {
    if (status_word_state_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No status_word.state_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "status_word.state_interface: " << status_word_state_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No status_word.state_interface parameter");
    exit(1);
  }
  if (this->get_parameter("mode_of_operation.controller_name", mode_of_operation_controller_name_))
  {
    if (mode_of_operation_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "mode_of_operation.controller_name: " << mode_of_operation_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation.controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("mode_of_operation.command_interface", mode_of_operation_command_interface_name_))
  {
    if (mode_of_operation_command_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation.command_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "mode_of_operation.command_interface: " << mode_of_operation_command_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation.command_interface parameter");
    exit(1);
  }
  if (this->get_parameter("mode_of_operation_display.controller_name", mode_of_operation_display_controller_name_))
  {
    if (mode_of_operation_display_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation_display.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "mode_of_operation_display.controller_name: " << mode_of_operation_display_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation_display.controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("mode_of_operation_display.state_interface", mode_of_operation_display_state_interface_name_))
  {
    if (mode_of_operation_display_state_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation_display.state_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "mode_of_operation_display.state_interface: " << mode_of_operation_display_state_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No mode_of_operation_display.state_interface parameter");
    exit(1);
  }

  // MOO
  rclcpp::CallbackGroup::SharedPtr cb_group_moo = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_moo;

  moo_publisher_ = this->create_publisher<ethercat_utils_msgs::msg::GPIOArray>(mode_of_operation_controller_name_ + "/commands", 1);

  moo_subscription_ = this->create_subscription<control_msgs::msg::InterfaceValue>(
        mode_of_operation_display_controller_name_ + "/inputs", 1, std::bind(&Cia402SlaveUtils::read_moo_cb, this, _1), sub_options);

  // CTRL WORD - STATUS WORD
  rclcpp::CallbackGroup::SharedPtr cb_group_word = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sub_options.callback_group = cb_group_word;

  reset_fault_server_ =
      this->create_service<std_srvs::srv::Trigger>("~/reset_fault",
                                                   std::bind(&Cia402SlaveUtils::reset_fault_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group_word);

  perform_homing_server_ =
      this->create_service<std_srvs::srv::Trigger>("~/perform_homing",
                                                   std::bind(&Cia402SlaveUtils::perform_homing_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group_word);

  start_position_control_server_ =
      this->create_service<std_srvs::srv::Trigger>("~/start_position_control",
                                                   std::bind(&Cia402SlaveUtils::start_position_control_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group_word);

  ctrl_word_publisher_ = this->create_publisher<ethercat_utils_msgs::msg::GPIOArray>(control_word_controller_name_ + "/commands", 1);

  status_word_subscription_ = this->create_subscription<control_msgs::msg::InterfaceValue>(
        status_word_controller_name_ + "/inputs", 1, std::bind(&Cia402SlaveUtils::read_status_cb, this, _1), sub_options);

  RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Services correctly initialized.");

  activate_jt_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
}

Cia402SlaveUtils::~Cia402SlaveUtils() {}

void Cia402SlaveUtils::reset_fault_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                      const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  {
    std::lock_guard<std::mutex> status_guard(status_word_mtx_);
    if (current_status_ != DeviceState::STATE_FAULT)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system is not in " << DEVICE_STATE_STR.at(DeviceState::STATE_FAULT) << ", but in " << DEVICE_STATE_STR.at(current_status_));
      res->success = true;
      res->message = "Fault Reset not performed. The system is not in " + DEVICE_STATE_STR.at(DeviceState::STATE_FAULT) + ", but in " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
  }

  if (!send_and_wait_for_control_word(134.0))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure. The system is in state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Fault Reset not performed. The system is in state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  if (current_status_ == DeviceState::STATE_SWITCH_ON_DISABLED)
  {
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "The system transitioned from FAULT to STATE_SWITCH_ON_DISABLED");
    double sec = 0.0;
    double timeout = 5;
    rclcpp::Rate rate(1000);
    while ( (current_status_ != DeviceState::STATE_READY_TO_SWITCH_ON) && sec < timeout)
    {
      sec += 0.001;
      rate.sleep();
    }
    if (sec >= timeout)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Wrong transition. Current state: " << DEVICE_STATE_STR.at(current_status_));
      res->success = false;
      res->message = "Wrong transition. Current state: " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "The system transitioned from STATE_SWITCH_ON_DISABLED to STATE_READY_TO_SWITCH_ON");
      res->success = true;
      res->message = "The system transitioned from FAULT to STATE_READY_TO_SWITCH_ON";
      return;
    }
  }
  else if (current_status_ == DeviceState::STATE_READY_TO_SWITCH_ON)
  {
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "The system transitioned from FAULT to STATE_READY_TO_SWITCH_ON");
    res->success = true;
    res->message = "The system transitioned from FAULT to STATE_READY_TO_SWITCH_ON";
    return;
  }

  RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Wrong transition to state: " << DEVICE_STATE_STR.at(current_status_));
  res->success = false;
  res->message = "Wrong transition to state: " + DEVICE_STATE_STR.at(current_status_);
  return;
}

void Cia402SlaveUtils::perform_homing_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                         const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  if (current_status_ == DeviceState::STATE_READY_TO_SWITCH_ON)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system is in " << DEVICE_STATE_STR.at(current_status_));
  }
  else
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system is in " << DEVICE_STATE_STR.at(current_status_));
    if ( send_and_wait_for_control_word(134.0) )
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system switch to state: " << DEVICE_STATE_STR.at(current_status_));
      if ((current_status_ != DeviceState::STATE_READY_TO_SWITCH_ON) && (current_status_ != DeviceState::STATE_SWITCH_ON_DISABLED))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "The system should be in state: " << DEVICE_STATE_STR.at(DeviceState::STATE_READY_TO_SWITCH_ON));
        res->success = false;
        res->message = "Homing process not performed. The system is NOT in STATE_READY_TO_SWITCH_ON state";
        return;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
      res->success = false;
      res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
  }

  if (!change_mode_of_operation(0))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure with moo change");
    res->success = false;
    res->message = "Failure with moo change";
    return;
  }

  if (send_and_wait_for_control_word(7.0))
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Current state: " << DEVICE_STATE_STR.at(current_status_));
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  if (send_and_wait_for_control_word(15.0))
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Current state: " << DEVICE_STATE_STR.at(current_status_));
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  if (send_and_wait_for_control_word(31.0))
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Current state: " << DEVICE_STATE_STR.at(current_status_));
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  rclcpp::Rate rate(1000);
  while ((current_status_word_ & 0b0001000000000000) != 0b0001000000000000)
  {
    if ((current_status_word_ & 0b0010000000000000) == 0b0010000000000000)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Error during homing process. Current state: " << DEVICE_STATE_STR.at(current_status_));
      res->success = false;
      res->message = "Error during homing process. Current state: " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
    if (current_status_ == DeviceState::STATE_FAULT)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Error during homing process. Current state: " << DEVICE_STATE_STR.at(current_status_));
      res->success = false;
      res->message = "Error during homing process. Current state: " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
    rate.sleep();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The homing process is finished. Current state: " << DEVICE_STATE_STR.at(current_status_));
  res->success = true;
  res->message = "The homing process is finished. Current state: " + DEVICE_STATE_STR.at(current_status_);
  homing_executed_ = true;
  return;
}

void Cia402SlaveUtils::start_position_control_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                                 const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!homing_executed_)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Homing process is required to start a control mode.");
    res->success = false;
    res->message = "Homing process is required to start a control mode.";
    return;
  }

  if (current_status_ == DeviceState::STATE_READY_TO_SWITCH_ON)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system is in " << DEVICE_STATE_STR.at(current_status_));
  }
  else
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system is in " << DEVICE_STATE_STR.at(current_status_));
    if ( send_and_wait_for_control_word(134.0) )
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The system switch to state: " << DEVICE_STATE_STR.at(current_status_));
      if ((current_status_ != DeviceState::STATE_READY_TO_SWITCH_ON) && (current_status_ != DeviceState::STATE_SWITCH_ON_DISABLED))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "The system should be in state: " << DEVICE_STATE_STR.at(DeviceState::STATE_READY_TO_SWITCH_ON));
        res->success = false;
        res->message = "Start position control process not performed. The system is NOT in STATE_READY_TO_SWITCH_ON state";
        return;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
      res->success = false;
      res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
      return;
    }
  }

  // Activate joint_trajectory_controller
  auto activation_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  activation_req->activate_controllers = {joint_trajectory_controller_name_};
  activation_req->strictness = 2; // fails if cannot activate one of the listed controllers

  auto timeout = builtin_interfaces::msg::Duration();
  timeout.set__sec(5);
  activation_req->timeout = timeout;

  using std::chrono_literals::operator""s;

  while (!activate_jt_controller_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger(this->get_name()), "Interrupted while waiting for the switch_jt_controller service. Exiting.");
      res->success = false;
      res->message = "Interrupted while waiting for the switch_jt_controller service.";
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "switch_jt_controller service not available, waiting again...");
  }

  auto activation_res = activate_jt_controller_client_->async_send_request(activation_req);
  activation_res.wait();

  if (activation_res.get()->ok) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "The " << joint_trajectory_controller_name_ << " has been correctly activated.");
  }
  else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failed to call activate " << joint_trajectory_controller_name_);
  }

  if (!change_mode_of_operation(1))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure with moo change");
    res->success = false;
    res->message = "Failure with moo change";
    return;
  }

  if (!send_and_wait_for_control_word(7.0))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  if (!send_and_wait_for_control_word(15.0))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure to change state. Current state: " << DEVICE_STATE_STR.at(current_status_));
    res->success = false;
    res->message = "Failure to change state. Current state: " + DEVICE_STATE_STR.at(current_status_);
    return;
  }

  res->success = true;
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Success." << DEVICE_STATE_STR.at(current_status_));
  return;
}

void Cia402SlaveUtils::read_moo_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg) {
  new_moo_msg_ = true;
  std::lock_guard<std::mutex> moo_guard(moo_mtx_);

  auto index = std::find(msg->interface_names.begin(), msg->interface_names.end(), mode_of_operation_display_state_interface_name_);
  if ( index != msg->interface_names.end())
  {
    current_moo_ = (ModeOfOperation)(msg->values[index - msg->interface_names.begin()]);
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), mode_of_operation_display_state_interface_name_ << " not found in "
                        << mode_of_operation_display_controller_name_ << " inputs");
  }
  return;
}

void Cia402SlaveUtils::read_status_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg) {
  std::lock_guard<std::mutex> status_guard(status_word_mtx_);
  auto index = std::find(msg->interface_names.begin(), msg->interface_names.end(), status_word_state_interface_name_);
  if ( index != msg->interface_names.end())
  {
    current_status_word_ = (uint16_t) msg->values[index - msg->interface_names.begin()];
    current_status_ = deviceState((uint16_t) msg->values[index - msg->interface_names.begin()]);
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), status_word_state_interface_name_ << " not found in "
                        << status_word_controller_name_ << " inputs");
  }
  return;
}

DeviceState Cia402SlaveUtils::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

bool Cia402SlaveUtils::send_and_wait_for_control_word(double ctrl_word) {
  DeviceState old_status = current_status_;
  int old_status_word = current_status_word_;
  double timeout = 5;
  double sec = 0;
  rclcpp::Rate rate(1000);

  ethercat_utils_msgs::msg::GPIOArray ctrl_word_msg;
  ctrl_word_msg.name.push_back(control_word_command_interface_name_);
  ctrl_word_msg.data.push_back(ctrl_word);

  ctrl_word_publisher_->publish(ctrl_word_msg);

  while (old_status == current_status_ && sec < timeout)
  {
    if ( (old_status == current_status_) && (old_status_word != current_status_word_))
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Status didn't change but status word changed.");
      return true;
    }
    rate.sleep();
    sec += 0.001;
  }
  if (sec >= timeout)
    return false;

  return true;
}

bool Cia402SlaveUtils::change_mode_of_operation(int moo)
{
  new_moo_msg_ = false;

  double timeout = 5;
  double sec = 0;
  rclcpp::Rate rate(10);

  while ((!new_moo_msg_) && sec < timeout)
  {
    rate.sleep();
    sec += 0.1;
  }
  if (sec >= timeout)
  {
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Failure. Waited too long for message.");
    return false;
  }

  ModeOfOperation old_moo = current_moo_;

  ModeOfOperation new_moo;
  std::string new_moo_str;

  switch ( moo )
  {
    case 0:
      new_moo = ModeOfOperation::MODE_HOMING;
      new_moo_str = "MODE_HOMING";
      break;
    case 1:
      new_moo = ModeOfOperation::MODE_CYCLIC_SYNC_POSITION;
      new_moo_str = "MODE_CYCLIC_SYNC_POSITION";
      break;
    default:
      new_moo = ModeOfOperation::MODE_NO_MODE;
      break;
  }

  if (old_moo == new_moo)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Success. Already in desired moo: " << new_moo_str);
    return true;
  }

  ethercat_utils_msgs::msg::GPIOArray new_moo_msg;
  new_moo_msg.name.push_back(mode_of_operation_command_interface_name_);
  new_moo_msg.data.push_back(new_moo);

  moo_publisher_->publish(new_moo_msg);

  sec = 0;
  while (old_moo == current_moo_ && sec < timeout)
  {
    rate.sleep();
    sec += 0.1;
  }

  std::lock_guard<std::mutex> moo_guard(moo_mtx_);
  if (old_moo == current_moo_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(this->get_name()), "Failure. Moo no changed");
    return false;
  }
  else if (current_moo_ == new_moo)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Success. Changed to " << new_moo_str);
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "Failure. Moo changed in a wrong mode." );
    return false;
  }
}

} // namespace ethercat_utils

