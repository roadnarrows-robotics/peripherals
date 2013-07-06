#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xbox_360.h"

void testCB(const std_msgs::String &f)
{
  ROS_INFO("here");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "xbox_360");
  ros::NodeHandle n;

  ros::Subscriber test = n.subscribe("joint_command",1, testCB);

  ROS_INFO("xbox 360 node is under dev...");

  ros::spin();
}
