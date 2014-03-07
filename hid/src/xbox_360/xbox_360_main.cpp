////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals Human Iterface Device Package
//
// Link:      https://github.com/roadnarrows-robotics/peripherals
//
// ROS Node:  xbox_360
//
// File:      xbox_360_main.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS xbox_360 node main.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/inotify.h>
#include <errno.h>
#include <string.h>

#include "ros/ros.h"

#define LOG
#define LOGMOD "xbox_360"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "rnr/hid/HIDXbox360.h"

#include "xbox_360.h"

using namespace std;
using namespace rnr;

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Node Specific Defines and Data.
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

//
// Data
//
const char *NodeName = "xbox_360";  ///< this ROS node's name
static int NotifyFd;      ///< notify fild descriptor
static int NotifyWdDev;   ///< notify slash dev watch descriptor


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// RoadNarrows Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Options
//
static int  OptsRcvTimeoutTh  = 0;                  ///< baud rate

/*!
 * \brief The package information.
 *
 * For ROS nodes, RN package information is equivalent to this ROS application
 * information.
 * */
static const PkgInfo_T PkgInfo =
{
  NodeName,                       ///< package name
  "1.1.0",                        ///< package version
  "2014.02.22",                   ///< date (and time)
  "2014",                         ///< year
  NodeName,                       ///< package full name
  "Daniel Packard, Robin Knight", ///< authors
  "RoadNarrows LLC",              ///< owner
  "(C) 2014 RoadNarrows LLC"      ///< disclaimer
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  "[ROSOPTIONS]",

  // synopsis
  "The %P ROS node provides ROS interfaces to the Xbox360 game controller.",
  
  // long_desc 
  "The %P ROS node provides ROS interfaces to the Xbox360 game controller. "
  "Both wired and wireless versions of the Xbox360 game controller are "
  "supported.",
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // --threshold, t
  {
    "threshold",          // long_opt
    't',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsRcvTimeoutTh,    // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<seconds>",          // arg_name
    "Set the Xbox360 receive timeout threshold. If no game controller updates "
    "are received for >= threshold seconds, the link state to the game "
    "controller is disabled. This only applies to wireless controllers.\n"
    "Set to 0 to disable thresholding."
                          // opt desc
  },

  {NULL, }
};


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Main Functions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Create and run ROS node.
 *
 * \param hidXbox   Opened Xbox360 hid instance.
 */
static void runNode(HIDXbox360 &hidXbox)
{
  string      strNodeName;  // ROS-given node name

  ros::NodeHandle nh(NodeName);

  // actual ROS-given node name
  strNodeName = ros::this_node::getName();

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    return;
  }

  ROS_INFO("%s: Node started.", strNodeName.c_str());

  //
  // Create the xbox_360 node object.
  //
  hid::Xbox360  xbox(nh, hidXbox);

  //
  // Advertise services.
  //
  xbox.advertiseServices();

  ROS_INFO("%s: Services registered.", strNodeName.c_str());

  //
  // Advertise publishers.
  //
  xbox.advertisePublishers(4);
  
  ROS_INFO("%s: Publishers registered.", strNodeName.c_str());
  
  //
  // Subscribed to topics.
  //
  xbox.subscribeToTopics(4);
  
  ROS_INFO("%s: Subscribed topics registered.", strNodeName.c_str());

  // set loop rate in Hertz
  ros::Rate loop_rate(30);

  ROS_INFO("%s: Ready.", strNodeName.c_str());

  //
  // Node loop.
  //
  while( xbox.isConnected() && ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all advertized topics
    xbox.publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }
}

/*!
 * \brief Initialize resource to watch for new plug-n-play controllers.
 *
 * The directory /dev is watched for new files (i.e. devices).
 *
 * \note No ROS node has been created.
 *
 * \return Returns 0 on success, \h_lt 0 on failure.
 */
static int initWatch()
{
  const char *sDev = "/dev";

  if( (NotifyFd = inotify_init()) < 0 )
  {
    LOGERROR("Failed to initialize inotify.");
    return -1;
  }

  NotifyWdDev = inotify_add_watch(NotifyFd, sDev, IN_CREATE);

  if( NotifyWdDev < 0 )
  {
    LOGERROR("Failed to add directory %s to inotify watch.", sDev);
    return -1;
  }

  return 0;
}

/*!
 * \brief Watch for new plug-n-play game Xbox360 controllers.
 *
 * \note No ROS node has been created.
 *
 * \return Returns 0 on success, \h_lt 0 on failure.
 */
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

  LOGDIAG2("Watching for new Xbox360 devices.");

  //
  // Watch loop.
  //
  while( true )
  {
    // block on read 
    if( read(NotifyFd, u.buf, sizeof(u.buf)) > 0 )
    {
      plugin = (struct inotify_event *)u.buf;
      LOGDIAG2("New device %s", plugin->name);
      if( strncmp(plugin->name, sPrefix, strlen(sPrefix)) == 0 )
      {
        LOGDIAG2("Got an Xbox360 controller.");
        return 0;
      }
    }
    
    // read error
    else
    {
      LOGSYSERROR("read on inotify");
      return -1;
    }
  }
}

/*!
 *  \brief ROS xbox_360 node main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns exit code.
 */
int main(int argc, char* argv[])
{
  HIDXbox360  hidXbox;      // Xbox360 hid

  // 
  // Initialize the node.
  //
  ros::init(argc, argv, "xbox_360");

  //
  // Parse node-specific options and arguments (from librnr).
  //
  OptsGet(NodeName, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);
 
  //
  // Initial resources to watch for new game controller devices.
  //
  if( initWatch() < 0 )
  {
    ROS_FATAL("Failed to initialize inotify watch.");
    return APP_EC_EXEC;
  }

  //
  // Daemon loop.
  //
  while( true )
  {
    //
    // Try to open up an Xbox360 device.
    //
    if( hidXbox.open() == 0 )
    {
      // Opened the device, create and run as the xbox_360 ROS node.
      runNode(hidXbox);

      // End of ROS node session, close.
      hidXbox.close();
    }

    //
    // Watch for new Xbox360 device to be plugged in.
    //
    else if( watch() < 0 )
    {
      // Watch failure, probably a signal.
      return APP_EC_EXEC;
    }
  }

  return APP_EC_OK;
}
