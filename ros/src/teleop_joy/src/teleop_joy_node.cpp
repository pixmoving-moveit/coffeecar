#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

class TeleopCoffeeRobot
{
public:
  TeleopCoffeeRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

TeleopCoffeeRobot::TeleopCoffeeRobot():
  linear_(1),
  angular_(3),
  a_scale_(0.44),  // TODO: use a parameter instead
  l_scale_(2.7)    // TODO: use a parameter instead
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopCoffeeRobot::joyCallback, this);
}

void TeleopCoffeeRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::TwistStamped twist;
  twist.twist.angular.z = a_scale_*joy->axes[angular_];
  twist.twist.linear.x = l_scale_*joy->axes[linear_];
  // ROS_INFO_STREAM("Angular " << joy->axes[angular_]);
  // ROS_INFO_STREAM("Linear " << joy->axes[linear_]);

  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_coffee_robot");
  TeleopCoffeeRobot teleop_coffee_robot;

  ros::spin();
}