#include <functional>
#include <memory>

#include <mqtt/client.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloSubscriber: public rclcpp::Node
{
public:
  HelloSubscriber()
  : Node("Hello_subscriber")
  {
    pMqttClient_ = new mqtt::client(MQTT_ADDRESS_, MQTT_CLIENT_ID_);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    pMqttClient_->connect(connOpts);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    hello_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "GREETING",
      qos_profile,
      std::bind(&HelloSubscriber::subscriber_topic_message, this, _1));
  }

  ~HelloSubscriber()
  {
    if(pMqttClient_)
    {
      pMqttClient_->disconnect();
      delete(pMqttClient_);
    }
  }

private:
  void subscriber_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());

    auto mqtt_msg = mqtt::make_message(MQTT_TOPIC_, msg->data.c_str());
    mqtt_msg->set_qos(MQTT_QOS_);
    pMqttClient_->publish(mqtt_msg);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hello_subscriber_;

  const std::string MQTT_ADDRESS_ {"tcp://localhost:1883"};
  const std::string MQTT_CLIENT_ID_ {"my_first_ros_hello"};
  const std::string MQTT_TOPIC_ {"ros/hello"};
  const int MQTT_QOS_ = 1;
  mqtt::client * pMqttClient_ = NULL;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HelloSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}