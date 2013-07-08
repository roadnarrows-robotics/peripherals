#ifndef _XBOX_360_SERVICES_H
#define _XBOX_360_SERVICES_H

#include "hid/SetRumble.h"
#include "xbox_360.h"

bool SetRumble(hid::SetRumble::Request &req,
               hid::SetRumble::Response &res)
{
  pXbox->setRumble(req.left_rumble, req.right_rumble);
  return true;
}

#endif // _XBOX_360_SERVICES_H
