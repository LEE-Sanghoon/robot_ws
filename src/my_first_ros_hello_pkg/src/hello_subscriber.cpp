#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloSubscriber: public rclcpp::Node
{
public:
  HelloSubscriber()
  : Node("Hello_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    hello_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "GREETING",
      qos_profile,
      std::bind(&HelloSubscriber::subscriber_topic_message, this, _1));
  }

private:
  void subscriber_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hello_subscriber_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HelloSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}