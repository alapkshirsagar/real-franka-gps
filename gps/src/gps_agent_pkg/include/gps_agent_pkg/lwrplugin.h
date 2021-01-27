/*****************************************************************************
 * KUKA LWR 4+ ROS ros_control plugin for GPS
 * Jack White, Aalto University 2018
 *****************************************************************************/

#ifndef LWR_PLUGIN_H
#define LWR_PLUGIN_H

#include <controller_interface/controller.h>
#include <lwr_hw/lwr_hw.h>
#include <cstdint>

#include <control_msgs/JointControllerState.h>

#include "gps_agent_pkg/singlearmplugin.h"
#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/controller.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/encodersensor.h"
#include "gps/proto/gps.pb.h"

namespace hw_ifc = hardware_interface;
namespace cn_ifc = controller_interface;

namespace gps_control
{
// Although based on the KUKA LWR 4+ controller code at
// https://github.com/CentroEPiaggio/kuka-lwr this controller does not use
// the derived KinematicChainControllerBase class given there, since GPS's
// own RobotPlugin class handles its own kinematic chains

class KUKALWRPlugin: public SingleArmPlugin,
  public cn_ifc::Controller<hw_ifc::EffortJointInterface>
{
protected:
  uint64_t controller_counter_; // Number of controller steps elapsed
  uint16_t controller_step_len_ms_; // Length of controller step in ms
  hw_ifc::EffortJointInterface* robot_; // Interface to the robot state
  std::vector<typename hw_ifc::EffortJointInterface::ResourceHandleType>
    joint_handles_;
  ros::Time last_update_time_;
  KDL::JntArray desired_torques_;

public:
  KUKALWRPlugin(); // Only inits controller variables in this class
  virtual ~KUKALWRPlugin();
  virtual bool init(hw_ifc::EffortJointInterface* robot,
                    ros::NodeHandle& n);
  virtual void starting(const ros::Time& time); // Manager calls bef. start
  virtual void stopping(const ros::Time& time); // Manager calls bef. stop
  virtual void update(const ros::Time& time,
                      const ros::Duration& period); // Core of the plugin
  virtual void get_joint_encoder_readings(Eigen::VectorXd &angles,
                                          gps::ActuatorType arm) const;

  // Does not return the current time, but the last update time
  virtual ros::Time get_current_time() const;
  /* The update() method must:
  - update sensors
  - update the position or trial controller
  - publish *at the end*
  */

};
}

#endif // LWR_PLUGIN_H