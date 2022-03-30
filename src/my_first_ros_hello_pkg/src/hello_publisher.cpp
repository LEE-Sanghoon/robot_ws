#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloPublisher: public rclcpp::Node
{
public:
  HelloPublisher()
  : Node("Hello_publisher"), count_(0)
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    hello_publisher_ = this->create_publisher<std_msgs::msg::String>("GREETING", qos_profile);
    timer_ = this->create_wall_timer(1s, std::bind(&HelloPublisher::publish_hello_msg, this));
  }

private:
  void publish_hello_msg()
  {
    auto msg = std_msgs::msg::String();

    msg.data = "Hello: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    hello_publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hello_publisher_;
  size_t count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HelloPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}