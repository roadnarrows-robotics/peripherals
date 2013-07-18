// header, etc
#include <string>
#include "ros/ros.h"
#include "mot/SetSpeed.h"
//#include "mot/SetCurrentLimits.h"
#include "roboteq_sm.h"
#include "rnr/mot/MotRoboteqSmall.h"

using namespace std;

bool SetSpeed(mot::SetSpeed::Request &req,
              mot::SetSpeed::Response &rsp)
//bool SetCurrentLimits(mot::SetCurrentLimits::Request &req,
//                      mot::SetCurrentLimits::Response &rsp)
                      
{
  ROS_INFO("Setting speed");
    pMot->setSpeed(req.mot_id,req.speed);
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "roboteq");
  ros::NodeHandle n("roboteq");

  pMot=new MotRoboteqSmall;

  string devName="/dev/ttyACM0";
  int baudRate=115200;
  if(pMot->open(devName, baudRate)!=0)
  {
    fprintf(stderr, "jnt failed to open\n");
    return -1;
  }

// TODO - create services, subscriptions, published topics, etc
  
  //
  // Services
  ros::ServiceServer set_speed_ser = n.advertiseService("set_speed",
                                                       SetSpeed);

//  ros::ServiceServer set_current_ser = n.advertiseService("set_current_lim",
//                                                         SetCurrentLimits);
  ros::spin();

  if(pMot->close()!=0)
  {
    fprintf(stderr, "jnt failed to close\n");
    return -1;
  }

  return 0;
}
