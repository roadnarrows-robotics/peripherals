#ifndef _XBOX_360_SERVICES_H
#define _XBOX_360_SERVICES_H

#include "hid/Ping.h"
#include "hid/SetLED.h"
#include "hid/SetRumble.h"

#include "xbox_360.h"

/*!
 * \brief Set the Rumble feature of the xBox controller 
 */
bool SetRumble(hid::SetRumble::Request &req,
               hid::SetRumble::Response &res)
{
  ROS_INFO("Setting Controller Rumble: %d %d", req.left_rumble, 
                                               req.right_rumble);
  res.rc = pXbox->setRumble(req.left_rumble, req.right_rumble);
  return true;
}

/*!
 * \brief Set the LED pattern feature of the xBox Controller
 */
bool SetLED(hid::SetLED::Request &req,
            hid::SetLED::Response &res)
{
  ROS_INFO("Setting Controller LED Pattern: %d", req.n);
  res.rc = pXbox->setLED(req.n);
  return true;
}

/*!
 * \brief Ping the xbox controller
 */
bool Ping(hid::Ping::Request &req,
          hid::Ping::Response &res)
{
  res.rc = pXbox->ping();
  return true;
}

#endif // _XBOX_360_SERVICES_H
