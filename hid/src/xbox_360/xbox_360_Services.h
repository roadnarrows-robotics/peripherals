#ifndef _XBOX_360_SERVICES_H
#define _XBOX_360_SERVICES_H

#include "hid/SetLED.h"
#include "hid/SetRumble.h"
#include "xbox_360.h"

/*!
 * \brief Set the Rumble feature of the xBox controller 
 */
bool SetRumble(hid::SetRumble::Request &req,
               hid::SetRumble::Response &res)
{
  res.rc = pXbox->setRumble(req.left_rumble, req.right_rumble);
  return true;
}

/*!
 * \brief Set the LED pattern feature of the xBox Controller
 */
bool SetLED(hid::SetLED::Request &req,
            hid::SetLED::Response &res)
{
  res.rc = pXbox->setLED(req.n);
  return true;
}

#endif // _XBOX_360_SERVICES_H
