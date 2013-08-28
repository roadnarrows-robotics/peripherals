#ifndef _ROBOTEQ_SM_SUBSCRITIONS_H
#define _ROBOTEQ_SM_SUBSCRITIONS_H

#include "ros/ros.h"
#include "mot/SpeedCmd.h"
#include "roboteq_sm.h"

#include <stdio.h>

void speed_commandCB(const mot::SpeedCmd &cmd)
{ 
  ROS_INFO("dhp -here cmd.speeds[0] = %d ",cmd.speeds[0] );
  if(cmd.mot_ids.size() != cmd.speeds.size())
  {
    ROS_ERROR("Size of mot ids does not match size of speeds: %lu != %lu", 
              cmd.mot_ids.size(), cmd.speeds.size());
    return;
  }

  pMot->setSpeed(cmd.mot_ids[0],cmd.speeds[0]);
  pMot->setSpeed(cmd.mot_ids[1],cmd.speeds[1]);

}


#endif // _ROBOTEQ_SM_SUBSCRITIONS_H
