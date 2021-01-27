/*
Acts the same as the PositionController class, except that the norm in the
is_finished() method excludes the last joint.
*/
#ifndef POSITION_CONTROLLER_LWR_HACK_H
#define POSITION_CONTROLLER_LWR_HACK_H

#include "gps_agent_pkg/positioncontroller.h"

namespace gps_control
{
  class PositionControllerLWRHack : public PositionController
  {
  public:
    PositionControllerLWRHack(ros::NodeHandle& n,
                              gps::ActuatorType arm,
                              int size);
    virtual bool is_finished() const; 
  };
}

#endif // POSITION_CONTROLLER_LWR_HACK_H