#ifndef _XBOX_360_SUBSCRIPTIONS_H
#define _XBOX_360_SUBSCRIPTIONS_H

#include "hid/RumbleCmd.h"

#include "xbox_360.h"

void rumble_commandCB(const hid::RumbleCmd &cmd)
{
  pXbox->setRumble(cmd.left_rumble, cmd.right_rumble);
  return;
}

#endif // _XBOX_360_SUBSCRIPTIONS_H
