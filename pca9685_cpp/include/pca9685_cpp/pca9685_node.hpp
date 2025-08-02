#ifndef PCA9685_CPP__PCA9685_NODE_HPP_
#define PCA9685_CPP__PCA9685_NODE_HPP_

#include "pca9685_cpp/pca9685_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>

namespace pca9685_cpp
{
class PCA9685Node : public rclcpp::Node
{
public:
  explicit PCA9685Node(const rclcpp::NodeOptions & options);

private:
  void on_pulse_received(const std_msgs::msg::Int32::SharedPtr msg, int channel);
  
  std::unique_ptr<PCA9685Driver> driver_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> subscriptions_;
};
}  // namespace pca9685_cpp

#endif  // PCA9685_CPP__PCA9685_NODE_HPP_
