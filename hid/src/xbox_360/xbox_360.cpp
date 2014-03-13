////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals Human Iterface Device Package
//
// Link:      https://github.com/roadnarrows-robotics/peripherals
//
// ROS Node:  xbox_360
//
// File:      xbox_360.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS xbox_360 node class implementation.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated hid messages.
//
#include "hid/ConnStatus.h"
#include "hid/Controller360State.h"

//
// ROS generatated hid services.
//
#include "hid/Ping.h"
#include "hid/SetLED.h"
#include "hid/SetRumble.h"
#include "hid/RumbleCmd.h"

//
// RoadNarrows embedded
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/hid/HIDXbox360.h"

//
// Node headers.
//
#include "xbox_360.h"


using namespace std;
using namespace rnr;
using namespace hid;


//------------------------------------------------------------------------------
// Xbox360 Class
//------------------------------------------------------------------------------

Xbox360::Xbox360(ros::NodeHandle &nh, HIDXbox360 &hidXbox) :
    m_nh(nh), m_hidXbox(hidXbox)
{
}

Xbox360::~Xbox360()
{
}


//..............................................................................
// Services
//..............................................................................

void Xbox360::advertiseServices()
{
  string  strSvc;

  strSvc = "ping_controller";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &Xbox360::ping,
                                          &(*this));

  strSvc = "set_led";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &Xbox360::setLED,
                                          &(*this));

  strSvc = "set_rumble";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &Xbox360::setRumble,
                                          &(*this));
}

bool Xbox360::ping(hid::Ping::Request  &req,
                   hid::Ping::Response &rsp)
{
  ROS_DEBUG("ping_controller");

  rsp.rc = m_hidXbox.ping();

  return true;
}

bool Xbox360::setLED(hid::SetLED::Request  &req,
                     hid::SetLED::Response &rsp)
{
  ROS_DEBUG("set_led");

  rsp.rc = m_hidXbox.setLED(req.led_pattern.val);

  ROS_INFO("Set LED pattern %d.", req.led_pattern.val);

  return true;
}

bool Xbox360::setRumble(hid::SetRumble::Request  &req,
                        hid::SetRumble::Response &rsp)
{
  ROS_DEBUG("set_rumble");

  rsp.rc = m_hidXbox.setRumble(req.left_rumble, req.right_rumble);
  
  ROS_INFO("Set Rumble motors %d, %d.", req.left_rumble,req.right_rumble);

  return true;
}


//..............................................................................
// Topic Publishers
//..............................................................................

void Xbox360::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "controller_360_state";
  m_publishers[strPub] =
    m_nh.advertise<hid::Controller360State>(strPub, nQueueDepth);

  strPub = "conn_status";
  m_publishers[strPub] = m_nh.advertise<hid::ConnStatus>(strPub, nQueueDepth);
}

void Xbox360::publish()
{
  publishXboxState();
  publishConnStatus();
}

void Xbox360::publishXboxState()
{
  int *pState = (int *)m_hidXbox.getCurrentState();

  m_msgXboxState.a_button          = pState[Xbox360FeatIdAButton];
  m_msgXboxState.b_button          = pState[Xbox360FeatIdBButton];
  m_msgXboxState.x_button          = pState[Xbox360FeatIdXButton];
  m_msgXboxState.y_button          = pState[Xbox360FeatIdYButton];

  m_msgXboxState.dpad_left         = pState[Xbox360FeatIdPadLeft];
  m_msgXboxState.dpad_right        = pState[Xbox360FeatIdPadRight];
  m_msgXboxState.dpad_up           = pState[Xbox360FeatIdPadUp];
  m_msgXboxState.dpad_down         = pState[Xbox360FeatIdPadDown];

  m_msgXboxState.back_button       = pState[Xbox360FeatIdBack];
  m_msgXboxState.center_button     = pState[Xbox360FeatIdCenterX];
  m_msgXboxState.start_button      = pState[Xbox360FeatIdStart];

  m_msgXboxState.left_joy_click    = pState[Xbox360FeatIdLeftStickClick];
  m_msgXboxState.left_joy_x        = pState[Xbox360FeatIdLeftJoyX];
  m_msgXboxState.left_joy_y        = pState[Xbox360FeatIdLeftJoyY];

  m_msgXboxState.right_joy_click   = pState[Xbox360FeatIdRightStickClick];
  m_msgXboxState.right_joy_x       = pState[Xbox360FeatIdRightJoyX];
  m_msgXboxState.right_joy_y       = pState[Xbox360FeatIdRightJoyY];

  m_msgXboxState.left_bump         = pState[Xbox360FeatIdLeftBump];
  m_msgXboxState.right_bump        = pState[Xbox360FeatIdRightBump];
  m_msgXboxState.left_trig         = pState[Xbox360FeatIdLeftTrigger];
  m_msgXboxState.right_trig        = pState[Xbox360FeatIdRightTrigger];

  // publish
  m_publishers["controller_360_state"].publish(m_msgXboxState);
}

void Xbox360::publishConnStatus()
{
  bool    was_connected = m_msgConnStatus.is_connected;
  bool    was_linked    = m_msgConnStatus.is_linked;

  m_msgConnStatus.is_connected  = m_hidXbox.isConnected();
  m_msgConnStatus.is_linked     = m_hidXbox.isLinked();

  if( (m_msgConnStatus.is_connected != was_connected) ||
      (m_msgConnStatus.is_linked != was_linked) )
  {
    ROS_INFO("Xbox360 %s, %s",
        (m_msgConnStatus.is_connected? "connected": "not connected"),
        (m_msgConnStatus.is_linked? "linked": "not linked"));
  }

  // publish
  m_publishers["conn_status"].publish(m_msgConnStatus);
}

void Xbox360::publishDisconnect()
{
  m_msgConnStatus.is_connected  = false;
  m_msgConnStatus.is_linked     = false;

  // publish
  m_publishers["conn_status"].publish(m_msgConnStatus);
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void Xbox360::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "rumble_command";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &Xbox360::execRumbleCmd,
                                          &(*this));
}

void Xbox360::execRumbleCmd(const hid::RumbleCmd &cmd)
{
  ROS_DEBUG("Executing rumble command.");

  m_hidXbox.setRumble(cmd.left_rumble, cmd.right_rumble);
}
