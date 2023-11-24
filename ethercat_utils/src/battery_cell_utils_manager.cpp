#include <ethercat_utils/battery_cell_utils.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ethercat_utils::BatteryCellUtils>("battery_cell_utils");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
