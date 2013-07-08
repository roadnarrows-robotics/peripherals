#ifndef _XBOX_360_STATEPUB_H
#define _XBOX_360_STATEPUB_H

#include "rnr/hid/HIDXbox360.h"
#include "hid/Controller360State.h"

#include "xbox_360.h"

using namespace rnr;

int updateController360State(hid::Controller360State  &s)
{
  int *pState = (int*)pXbox->getCurrentState();

  s.a_button          = pState[Xbox360FeatIdAButton];
  s.b_button          = pState[Xbox360FeatIdBButton];
  s.x_button          = pState[Xbox360FeatIdXButton];
  s.y_button          = pState[Xbox360FeatIdYButton];

  s.dpad_left         = pState[Xbox360FeatIdPadLeft];
  s.dpad_right        = pState[Xbox360FeatIdPadRight];
  s.dpad_up           = pState[Xbox360FeatIdPadUp];
  s.dpad_down         = pState[Xbox360FeatIdPadDown];

  s.back_button       = pState[Xbox360FeatIdBack];
  s.center_button     = pState[Xbox360FeatIdCenterX];
  s.start_button      = pState[Xbox360FeatIdStart];

  s.left_joy_click    = pState[Xbox360FeatIdLeftStickClick];
  s.left_joy_x        = pState[Xbox360FeatIdLeftJoyX];
  s.left_joy_y        = pState[Xbox360FeatIdLeftJoyY];

  s.right_joy_click   = pState[Xbox360FeatIdRightStickClick];
  s.right_joy_x       = pState[Xbox360FeatIdRightJoyX];
  s.right_joy_y       = pState[Xbox360FeatIdRightJoyY];

  s.left_bump         = pState[Xbox360FeatIdLeftBump];
  s.right_bump        = pState[Xbox360FeatIdRightBump];
  s.left_trig         = pState[Xbox360FeatIdLeftTrigger];
  s.right_trig        = pState[Xbox360FeatIdRightTrigger];
}

#endif // _XBOX_360_STATEPUB_H
