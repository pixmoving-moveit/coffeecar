#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_bridge/topic_to_socketcan.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv, "can_send");
	ros::NodeHandle nh;
	
	ros::Publisher publisher_ = nh.advertise<can_msgs::Frame>("sent_messages", 10);
	
	ros::Rate loop_rate(10);
	
	int count = 0;
	while(ros::ok())
	{
  can_msgs::Frame msg;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.is_error = false;
  msg.id = 0x0183;
  msg.dlc = 8;
  msg.data[0] = 100;

  msg.header.frame_id = "0";  // "0" for no frame.
  msg.header.stamp = ros::Time::now();

  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);
  
  ros::spinOnce();
  loop_rate.sleep();
  count++;
 

}
return 0;
}
