#include "gps_agent_pkg/positioncontrollerlwrhack.h"
#include "gps_agent_pkg/positioncontroller.h"

using namespace gps_control;

bool PositionControllerLWRHack::is_finished() const
{
    // Check whether we are close enough to the current target.
    if (mode_ == gps::JOINT_SPACE){
        // double epspos = 0.185;
        // double epsvel = 0.01;
        double epspos = 0.385;
        double epsvel = 0.03;
        int ang_size = current_angles_.size();
        double error = (current_angles_.head(ang_size - 1) -
                        target_angles_.head(ang_size - 1)).norm();
        double vel = current_angle_velocities_.norm();
        // ROS_INFO_STREAM("PosCn finished? poserr: " << error << " vel: " <<
        //                 vel << " T/H: " << epspos << "/" << epsvel);
        return (error < epspos && vel < epsvel);
    }
    else if (mode_ == gps::NO_CONTROL){
        return true;
    }
}


PositionControllerLWRHack::PositionControllerLWRHack(ros::NodeHandle& n,
                                                     gps::ActuatorType arm,
                                                     int size) :
  PositionController(n, arm, size)
{
  // Empty
}