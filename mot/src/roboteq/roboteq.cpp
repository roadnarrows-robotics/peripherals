// header, etc
#include "ros/ros.h"
#include "roboteq.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "roboteq");
  ros::NodeHandle n("roboteq");

// TODO - create services, subscriptions, published topics, etc

  ros::spin();

  return 0;
}
