#include "pca9685_cpp/pca9685_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pca9685_cpp
{

PCA9685Node::PCA9685Node(const rclcpp::NodeOptions & options)
: Node("pca9685_node", options)
{
  // Declare and retrieve parameters
  int bus = this->declare_parameter<int>("bus", 1);
  int address = this->declare_parameter<int>("address", 0x40);
  int frequency = this->declare_parameter<int>("frequency", 60);

  RCLCPP_INFO(this->get_logger(), "Initializing PCA9685 on I2C bus %d at address 0x%X", bus, address);

  // Initialize driver
  try {
    driver_ = std::make_unique<PCA9685Driver>(bus, address);
    driver_->set_pwm_frequency(frequency);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize PCA9685 driver: %s", e.what());
    rclcpp::shutdown();
    return;
  }
  
  // Create subscriptions for 16 channels
  constexpr int number_of_channels = 16;
  for (int i = 0; i < number_of_channels; ++i) {
    auto topic_name = "/pwm_channel_" + std::to_string(i);
    subscriptions_.push_back(
      this->create_subscription<std_msgs::msg::Int32>(
        topic_name, 10,
        [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
          this->on_pulse_received(msg, i);
        }));
  }
  RCLCPP_INFO(this->get_logger(), "Subscribed to %d PWM channels.", number_of_channels);
}

void PCA9685Node::on_pulse_received(const std_msgs::msg::Int32::SharedPtr msg, int channel)
{
  int pulse = msg->data;
  if (pulse < 0 || pulse > 4095) {
      RCLCPP_WARN(this->get_logger(), 
        "Pulse value %d is out of range [0, 4095] for channel %d.", pulse, channel);
  }
  try {
    driver_->set_pulse(channel, pulse);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error setting pulse for channel %d: %s", channel, e.what());
  }
}

} // namespace pca9685_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(pca9685_cpp::PCA9685Node)
