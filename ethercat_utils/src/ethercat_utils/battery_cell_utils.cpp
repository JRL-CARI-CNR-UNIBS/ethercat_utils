#include <ethercat_utils/battery_cell_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace ethercat_utils {

BatteryCellUtils::BatteryCellUtils(const std::string name,
                                   const rclcpp::NodeOptions option)
    : rclcpp::Node(name, option) {

  if (this->get_parameter("lubrication.command_interface", lubrication_command_interface_name_))
  {
    if (lubrication_command_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.command_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "lubrication.command_interface: " << lubrication_command_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.command_interface parameter");
    exit(1);
  }
  if (this->get_parameter("lubrication.command_controller_name", lubrication_command_controller_name_))
  {
    if (lubrication_command_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.command_controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "lubrication.command_controller_name: " << lubrication_command_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.command_controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("lubrication.state_interface", lubrication_state_interface_name_))
  {
    if (lubrication_state_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.state_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "lubrication.state_interface: " << lubrication_state_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.state_interface parameter");
    exit(1);
  }
  if (this->get_parameter("lubrication.state_controller_name", lubrication_state_controller_name_))
  {
    if (lubrication_state_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.state_controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "lubrication.state_controller_name: " << lubrication_state_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No lubrication.state_controller_name parameter");
    exit(1);
  }

  if (this->get_parameter("pneumatic_gripper.controller_name", pneumatic_gripper_controller_name_))
  {
    if (pneumatic_gripper_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No pneumatic_gripper.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "pneumatic_gripper.controller_name: " << pneumatic_gripper_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No pneumatic_gripper.opening_controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("pneumatic_gripper.command_interface", pneumatic_gripper_command_interface_name_))
  {
    if (pneumatic_gripper_command_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No pneumatic_gripper.command_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "pneumatic_gripper.command_interface: " << pneumatic_gripper_command_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No pneumatic_gripper.command_interface parameter");
    exit(1);
  }

  if (this->get_parameter("ati_sensor.controller_name", ati_sensor_controller_name_))
  {
    if (ati_sensor_controller_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.controller_name parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "ati_sensor.controller_name: " << ati_sensor_controller_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.opening_controller_name parameter");
    exit(1);
  }
  if (this->get_parameter("ati_sensor.command_interface", ati_sensor_command_interface_name_))
  {
    if (ati_sensor_command_interface_name_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.command_interface parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "ati_sensor.command_interface: " << ati_sensor_command_interface_name_);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.command_interface parameter");
    exit(1);
  }
  if (this->get_parameter("ati_sensor.state_interfaces", ati_sensor_state_interface_names_))
  {
    if (ati_sensor_state_interface_names_.empty()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.state_interfaces parameter, empty");
      exit(1);
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "ati_sensor.state_interfaces: ");
      for (auto ati_sensor_state_interface_name: ati_sensor_state_interface_names_)
        RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "           " << ati_sensor_state_interface_name);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), "No ati_sensor.state_interfaces parameter");
    exit(1);
  }

  rclcpp::CallbackGroup::SharedPtr cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group;

  lubrication_publisher_ = this->create_publisher<ethercat_utils_msgs::msg::GPIOArray>(lubrication_command_controller_name_ + "/commands", 1);
  pneumatic_gripper_publisher_ = this->create_publisher<ethercat_utils_msgs::msg::GPIOArray>(pneumatic_gripper_controller_name_ + "/commands", 1);
  ati_sensor_publisher_ = this->create_publisher<ethercat_utils_msgs::msg::GPIOArray>(ati_sensor_controller_name_ + "/commands", 1);

  lubrication_subscription_ = this->create_subscription<control_msgs::msg::InterfaceValue>(
        lubrication_state_controller_name_ + "/inputs", 1, std::bind(&BatteryCellUtils::read_lubrication_state_cb, this, _1), sub_options);

  ati_sensor_subscription_ = this->create_subscription<control_msgs::msg::InterfaceValue>(
        ati_sensor_controller_name_ + "/inputs", 1, std::bind(&BatteryCellUtils::read_ati_sensor_values_cb, this, _1), sub_options);

  lubrication_server_ =
      this->create_service<std_srvs::srv::Trigger>("~/lubrication",
                                                   std::bind(&BatteryCellUtils::lubrication_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group);
  pneumatic_gripper_server_ =
      this->create_service<std_srvs::srv::SetBool>("~/pneumatic_gripper",
                                                   std::bind(&BatteryCellUtils::pneumatic_gripper_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group);
  ati_sensor_server_ =
      this->create_service<std_srvs::srv::Trigger>("~/reset_ati_sensor",
                                                   std::bind(&BatteryCellUtils::reset_ati_sensor_cb,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2),
                                                   rmw_qos_profile_default,
                                                   cb_group);
}

BatteryCellUtils::~BatteryCellUtils() {}

void BatteryCellUtils::read_lubrication_state_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg)
{
  new_lubrication_msg_ = true;
  std::lock_guard<std::mutex> lubrication_guard(lubrication_mtx_);
  auto index = std::find(msg->interface_names.begin(), msg->interface_names.end(), lubrication_state_interface_name_);
  if ( index !=  msg->interface_names.end() )
  {
    lubrication_state_ = msg->values[index - msg->interface_names.begin()];
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), lubrication_state_interface_name_ << " not found in "
                        << lubrication_state_controller_name_ << " inputs");
  }
  return;
}

void BatteryCellUtils::read_ati_sensor_values_cb(const control_msgs::msg::InterfaceValue::SharedPtr msg)
{
  new_ati_sensor_msg_ = true;
  std::lock_guard<std::mutex> sensor_guard(ati_sensor_mtx_);
  ati_sensor_values_.clear();

  for (auto name: ati_sensor_state_interface_names_)
  {
    auto index = std::find(msg->interface_names.begin(), msg->interface_names.end(), name);
    if ( index !=  msg->interface_names.end() )
    {
      ati_sensor_values_.push_back(msg->values[index - msg->interface_names.begin()]);
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->get_name()), name << " not found in "
                          << ati_sensor_controller_name_ << " inputs");
    }
  }
  return;
}

void BatteryCellUtils::lubrication_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  ethercat_utils_msgs::msg::GPIOArray lubrication_msg;
  lubrication_msg.name.push_back(lubrication_command_interface_name_);
  lubrication_msg.data.push_back(1);

  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Writing on " << lubrication_command_controller_name_ << "/commands:  "
                      << lubrication_command_interface_name_ << " <- 1");

  using std::chrono_literals::operator""s;
  for (std::size_t i = 0; i < 3; i++)
  {
    lubrication_publisher_->publish(lubrication_msg);
    rclcpp::sleep_for(1s);
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), "Writing on " << lubrication_command_controller_name_ << "/commands:  "
                      << lubrication_command_interface_name_ << " <- 0");

  lubrication_msg.data.clear();
  lubrication_msg.data.push_back(0);
  for (std::size_t i = 0; i < 3; i++)
  {
    lubrication_publisher_->publish(lubrication_msg);
    rclcpp::sleep_for(1s);
  }

  bool end = false;

  double freq = 100;
  rclcpp::Rate rate(freq);
  double timeout = 180;
  double time = 0;

  while (!end && (time < timeout))
  {
    end = lubrication_state_;
    rate.sleep();
    time += (1/freq);
  }

  if ( time >= timeout)
    res->success = false;
  else
    res->success = true;

  return;
}

void BatteryCellUtils::pneumatic_gripper_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
                                            const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  ethercat_utils_msgs::msg::GPIOArray msg;
  if (req->data)
  {
    msg.data.push_back(0);
  }
  else
  {
    msg.data.push_back(1);
  }
  msg.name.push_back(pneumatic_gripper_command_interface_name_);
  pneumatic_gripper_publisher_->publish(msg);
  res->success = true;
}

void BatteryCellUtils::reset_ati_sensor_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                           const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  ethercat_utils_msgs::msg::GPIOArray msg;

  msg.data.push_back(1);
  msg.name.push_back(ati_sensor_command_interface_name_);

  bool zeroed = false;
  double freq = 100;
  rclcpp::Rate rate(freq);
  double timeout = 4;
  double time = 0;
  while (!zeroed && (time < timeout))
  {
    ati_sensor_publisher_->publish(msg);
    {
      std::lock_guard<std::mutex> sensor_guard(ati_sensor_mtx_);
      if (std::fabs(ati_sensor_values_[0]) < 1 &&
          std::fabs(ati_sensor_values_[1]) < 1 &&
          std::fabs(ati_sensor_values_[2]) < 1 &&
          std::fabs(ati_sensor_values_[3]) < 1 &&
          std::fabs(ati_sensor_values_[4]) < 1 &&
          std::fabs(ati_sensor_values_[5]) < 1)
      {
        zeroed = true;
      }
    }
    rate.sleep();
    time += (1.0/freq);
    std::cout << time << std::endl;
    std::cout << " " << std::endl;
    std::cout << std::fabs(ati_sensor_values_[0]) << std::endl;
    std::cout << std::fabs(ati_sensor_values_[1]) << std::endl;
    std::cout << std::fabs(ati_sensor_values_[2]) << std::endl;
    std::cout << std::fabs(ati_sensor_values_[3]) << std::endl;
    std::cout << std::fabs(ati_sensor_values_[4]) << std::endl;
    std::cout << std::fabs(ati_sensor_values_[5]) << std::endl;
  }

  zeroed = false;

  if ( time >= timeout)
    res->success = false;
  else
    res->success = true;

  return;
}

} // namespace ethercat_utils

