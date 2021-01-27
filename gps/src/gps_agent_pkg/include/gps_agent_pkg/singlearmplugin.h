/*******************************************************************************
  * singlearmplugin.h
  * by Jack White
  *
  * Class SingleArmPlugin acts as a single-arm RobotPlugin derivative for the
  * purpose of hoodwinking the Sensor class, which needs the two-arm RobotPlugin
  *
 ******************************************************************************/


#pragma once

#include "gps_agent_pkg/robotplugin.h"

#define SINGLE_ARM_CLASS_NAME_TO_PRNT "SingleArmPlugin: "

namespace gps_control
{

class SingleArmPlugin : public RobotPlugin
{
public:
  virtual ~SingleArmPlugin();
  virtual void initialize_sensors(ros::NodeHandle& n);
  virtual void initialize_sample(boost::scoped_ptr<Sample>& sample,
                                 gps::ActuatorType actuator_type);
  virtual void configure_sensors(OptionsMap &opts);
  virtual void update_sensors(ros::Time current_time, bool is_controller_step);
  virtual void update_controllers(ros::Time current_time,
                                  bool is_controller_step);
  virtual void position_subscriber_callback(const
      gps_agent_pkg::PositionCommand::ConstPtr& msg);
  virtual void relax_subscriber_callback(const
      gps_agent_pkg::RelaxCommand::ConstPtr& msg);
  virtual Sensor *get_sensor(SensorType sensor, gps::ActuatorType actuator_type);
  virtual void initialize_position_controllers(ros::NodeHandle& n);
};

}