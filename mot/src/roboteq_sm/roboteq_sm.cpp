// header, etc
#include <string>
#include "ros/ros.h"
#include "mot/SetSpeed.h"
#include "mot/SpeedCmd.h"
#include "roboteq_sm.h"
#include "roboteq_sm_Subscriptions.h"
#include "rnr/mot/MotRoboteqSmall.h"
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

using namespace std;

bool SetSpeed(mot::SetSpeed::Request &req,
              mot::SetSpeed::Response &rsp)
                      
{
  ROS_INFO("Setting speed");
    pMot->setSpeed(req.mot_id,req.speed);
    fprintf (stderr, "Speed = %f \n", req.speed);
  return true;
}

int main(int argc, char* argv[])
{
  // set loglevel for RN libs
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);
  ros::init(argc, argv, "roboteq");
  ros::NodeHandle n("roboteq");

  pMot=new MotRoboteqSmall;

  string devName1="/dev/ttyACM0";
  string devName2="/dev/ttyACM1";
  int baudRate=115200;
  if(pMot->open(devName1, baudRate)!=0)
  {
    fprintf(stderr, "jnt Motor 1 failed to open\n");
    return -1;
  }
  if(pMot->open(devName2, baudRate)!=0)
  {
    fprintf(stderr, "jnt Motor 2 failed to open\n");
    return -1;
  }

// TODO - create services, subscriptions, published topics, etc
  
  //
  // Services
  ros::ServiceServer set_speed_ser = n.advertiseService("set_speed",
                                                       SetSpeed);

  //
  // Subscriptions
  ros::Subscriber speed_command_sub = n.subscribe("speed_command", 1,
                                                  speed_commandCB);

  ros::spin();

  if(pMot->close()!=0)
  {
    fprintf(stderr, "jnt failed to close\n");
    return -1;
  }

  return 0;  

}
