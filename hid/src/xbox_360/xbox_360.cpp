#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xbox_360.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "xbox_360");
  ros::NodeHandle n;

  ros::Publisher test = n.advertise<std_msgs::String>("chitchat", 1000);

  ros::Rate loop_rate(10);

  ROS_INFO("xbox 360 node is under dev...");
  while(ros::ok())
  {
    std_msgs::String msg;
    msg.data="hello";
    test.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
