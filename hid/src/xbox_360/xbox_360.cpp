#include <sys/inotify.h>
#include <errno.h>
#include <string.h>

#include "ros/ros.h"

#define LOG
#define LOGMOD "xbox_360"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/hid/HIDXbox360.h"

#include "hid/Controller360State.h"
#include "hid/SetRumble.h"
#include "hid/ConnStatus.h"

#include "xbox_360_Services.h"
#include "xbox_360_Subscriptions.h"
#include "xbox_360_StatePub.h"

#include "xbox_360.h"

using namespace rnr;

static int NotifyFd;      ///< notify fild descriptor
static int NotifyWdDev;   ///< notify slash dev watch descriptor

static void runNode(HIDXbox360 &xbox)
{
  ros::NodeHandle n("xbox_360");

  //
  // Published Topics
  ros::Publisher controller_360_state_pub =
    n.advertise<hid::Controller360State>("controller_360_state", 1);

  ros::Publisher controller_360_conn_pub =
    n.advertise<hid::ConnStatus>("conn_status", 1);

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
  hid::ConnStatus msgConnStatus;

  ROS_INFO("xbox_360 ready for action!");
  ros::Rate loop_rate(30);

  xbox.run();

  while( xbox.isConnected() && ros::ok())
  {
    updateController360State(s);
    controller_360_state_pub.publish(s);

    msgConnStatus.is_connected = pXbox->isConnected();
    msgConnStatus.is_linked = pXbox->isLinked();
    controller_360_conn_pub.publish(msgConnStatus);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

static int initWatch()
{
  const char *sDev = "/dev";

  if( (NotifyFd = inotify_init()) < 0 )
  {
    ROS_FATAL("Failed to initialize inotify.");
    return -1;
  }

  NotifyWdDev = inotify_add_watch(NotifyFd, sDev, IN_CREATE);

  if( NotifyWdDev < 0 )
  {
    ROS_FATAL("Failed to add directory %s to watch.", sDev);
    return -1;
  }

  return 0;
}

static int watch()
{
  const char *sPrefix = "xbox360";

  // aligned read buffer
  union
  {
    void   *p;          // force alignment (for bad compilers)
    byte_t  buf[4096];  // the read buffer
  } u;

  struct inotify_event *plugin;     // create (plug-in) event

  fprintf(stderr, "watch()\n");

  while( true )
  {
    if( read(NotifyFd, u.buf, sizeof(u.buf)) > 0 )
    {
      plugin = (struct inotify_event *)u.buf;
      fprintf(stderr, "plugin name=%s\n", plugin->name);
      if( strncmp(plugin->name, sPrefix, strlen(sPrefix)) == 0 )
      {
        fprintf(stderr, "got it\n");
        return 0;
      }
    }
    else
    {
      fprintf(stderr, "read() error\n");
      return -1;
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "xbox_360");

  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  if( initWatch() < 0 )
  {
    ROS_FATAL("Failed to initialize inotify watch.");
    return 4;
  }

  // initialize global controll interface
  pXbox = new HIDXbox360();

  while( true )
  {
    if( pXbox->open() == 0 )
    {
      runNode(*pXbox);
      pXbox->close();
    }
    else if( watch() < 0 )
    {
      return 4;
    }
  }

  delete pXbox;

  return 0;
}
