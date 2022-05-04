#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mqtt/client.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

const std::string MQTT_ADDRESS_ 	{"tcp://localhost:1883"} ;
const std::string MQTT_CLIENT_ID_ {""} ;

const std::string MQTT_TOPIC_ 		{"ros/hello"} ;
const int MQTT_QOS_ = 1;

mqtt::client * pMqttClient_ = NULL ;

void linkerCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	auto mqtt_msg = mqtt::make_message(MQTT_TOPIC_, msg->data.c_str());
  mqtt_msg->set_qos(MQTT_QOS_);
  pMqttClient_->publish(mqtt_msg);
}

int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "rm_linker");

	pMqttClient_ = new mqtt::client(MQTT_ADDRESS_, MQTT_CLIENT_ID_);

	mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  connOpts.set_clean_session(true);

	try {
		pMqttClient_->connect(connOpts);
	}
	catch (const mqtt::exception& e) {
		std::cerr << "\nERROR: Unable to connect. " << e.what() << std::endl;
		return 1;
	}
  	

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber sub = n.subscribe("to_dg", 1000, linkerCallback);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */


	ros::spin();

	pMqttClient_->disconnect();
    delete(pMqttClient_);

	return 0;
}