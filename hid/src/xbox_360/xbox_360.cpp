#include "ros/ros.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/hid/HIDXbox360.h"
#include "hid/Controller360State.h"
#include "hid/SetRumble.h"

#include "xbox_360_Services.h"
#include "xbox_360_Subscriptions.h"
#include "xbox_360_StatePub.h"

#include "xbox_360.h"

using namespace rnr;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "xbox_360");
  ros::NodeHandle n;

  // initialize global controll interface
  pXbox = new HIDXbox360();
  if( pXbox->open() < 0 )
  {
    ROS_FATAL("Failed to open Xbox 360 controller.");
    return -1;
  }
  else if( !pXbox->ping() )
  {
    ROS_FATAL("Unable to ping Xbox 360 controller.");
    return -1;
  }

  //
  // Published Topics
  ros::Publisher controller_360_state_pub =
    n.advertise<hid::Controller360State>("controller_360_state", 1);


  // 
  // services
  ros::ServiceServer set_rumble_ser   = n.advertiseService("set_rumble",
                                                           SetRumble);

  ros::ServiceServer set_led_ser      = n.advertiseService("set_led",
                                                           SetLED);

  ros::ServiceServer ping_contr_ser   = n.advertiseService("ping_controller",
                                                           Ping);

  // 
  // subsrciptions
  ros::Subscriber rumble_cmd_sub      = n.subscribe("rumble_command", 1,
                                                    rumble_commandCB);

  //
  // published state data
  hid::Controller360State s;

  ROS_INFO("xbox_360 ready for action!");
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
    pXbox->update();
    updateController360State(s);
    controller_360_state_pub.publish(s);
    ros::spinOnce();
    loop_rate.sleep();
  }

  pXbox->close();
  delete pXbox;

  return 0;
}
