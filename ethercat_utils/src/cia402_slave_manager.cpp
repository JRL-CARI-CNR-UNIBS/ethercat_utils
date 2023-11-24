#include <ethercat_utils/cia402_slave_utils.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ethercat_utils::Cia402SlaveUtils>("cia402_slave_utils");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
