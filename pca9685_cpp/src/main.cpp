#include "rclcpp/rclcpp.hpp"
#include "pca9685/pca9685_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<pca9685::PCA9685Node>(options);
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
