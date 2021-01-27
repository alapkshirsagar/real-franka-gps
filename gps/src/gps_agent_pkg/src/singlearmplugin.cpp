/*******************************************************************************
  * singlearmplugin.cpp
  * by Jack White
  *
  * Class SingleArmPlugin acts as a single-arm RobotPlugin derivative for the
  * purpose of hoodwinking the Sensor class, which needs the two-arm RobotPlugin
  *
 ******************************************************************************/



#include "gps_agent_pkg/singlearmplugin.h"
#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/sensor.h"
#include "gps_agent_pkg/controller.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/positioncontrollerlwrhack.h"
#include "gps_agent_pkg/lingausscontroller.h"
#include "gps_agent_pkg/trialcontroller.h"
#include "gps_agent_pkg/LinGaussParams.h"
#include "gps_agent_pkg/tfcontroller.h"
#include "gps_agent_pkg/TfParams.h"
#include "gps_agent_pkg/ControllerParams.h"
#include "gps_agent_pkg/util.h"
#include "gps/proto/gps.pb.h"
#include <vector>

using namespace gps_control;

SingleArmPlugin::~SingleArmPlugin()
{
  // Empty
}


void
SingleArmPlugin::initialize_sensors(ros::NodeHandle& n)
{
  sensors_.clear(); // Remove old sensors

  // Create new sensors for ROS topics and encoders
  ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT << "Creating encoder sensor");
  boost::shared_ptr<Sensor> sensor1(Sensor::create_sensor(EncoderSensorType, n,
                                    this, gps::TRIAL_ARM));
  ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT << "Creating ROS topic sensor");
  boost::shared_ptr<Sensor> sensor2(Sensor::create_sensor(ROSTopicSensorType, n,
                                    this, gps::TRIAL_ARM));
  sensors_.push_back(sensor1);
  sensors_.push_back(sensor2);

  // Get current state from sensors
  current_time_step_sample_.reset(new Sample(MAX_TRIAL_LENGTH));
  initialize_sample(current_time_step_sample_, gps::TRIAL_ARM);

  sensors_initialized_ = true;
}


void
SingleArmPlugin::initialize_sample(boost::scoped_ptr<Sample>& sample,
                                   gps::ActuatorType actuator_type)
{
  if (actuator_type == gps::TRIAL_ARM)
    {
      std::cout << "Sensor size: " << sensors_.size() << std::endl;
      for (int i(0); i < sensors_.size(); i++)
        sensors_[i]->set_sample_data_format(sample);

      OptionsMap sample_metadata;
      sample->set_meta_data(gps::ACTION, active_arm_torques_.size(),
                            SampleDataFormatEigenVector, sample_metadata);
    }
  else if (actuator_type == gps::AUXILIARY_ARM)
    {
      ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT <<
                      "Passive arm requested on sample init");
    }
  else
    {
      ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT <<
                      "Unknown arm requested on sample init");
    }
  ROS_INFO("set sample data format");
}


void
SingleArmPlugin::configure_sensors(OptionsMap &opts)
{
  ROS_INFO("configure sensors");
  sensors_initialized_ = false;
  for (int i(0); i < sensors_.size(); i++)
    {
      sensors_[i]->configure_sensor(opts);
      sensors_[i]->set_sample_data_format(current_time_step_sample_);
    }

  OptionsMap sample_metadata;
  current_time_step_sample_->set_meta_data(gps::ACTION,
      active_arm_torques_.size(), SampleDataFormatEigenVector, sample_metadata);
  sensors_initialized_ = true;
}


void
SingleArmPlugin::update_sensors(ros::Time current_time, bool is_controller_step)
{
  if (!sensors_initialized_)
    return;

  // Update sensors and get sample
  for (int i(0); i < sensors_.size(); i++)
    {
      sensors_[i]->update(this, current_time, is_controller_step);
      if (trial_controller_ != NULL)
        {
          sensors_[i]->set_sample_data(current_time_step_sample_,
                                       trial_controller_->get_step_counter());
        }
      else
        {
          sensors_[i]->set_sample_data(current_time_step_sample_, 0);
        }
    }

  // Publish sample if requested
  if (trial_data_request_waiting_)
    {
      publish_sample_report(current_time_step_sample_);
      trial_data_request_waiting_ = false;
    }

  if (aux_data_request_waiting_)
    {
      ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT << "Aux data requested");
      aux_data_request_waiting_ = false;
    }
}


void
SingleArmPlugin::update_controllers(ros::Time current_time,
                                    bool is_controller_step)
{
  bool trial_init = trial_controller_ != NULL
                    && trial_controller_->is_configured()
                    && controller_initialized_;
  if (!is_controller_step && trial_init)
    return;

  if (trial_init)
    trial_controller_->update(this, current_time, current_time_step_sample_,
                              active_arm_torques_);
  else
    active_arm_controller_->update(this, current_time,
                                   current_time_step_sample_,
                                   active_arm_torques_);

  // Delete the trial controller if it is finished
  if (trial_init && trial_controller_->is_finished())
    {
      publish_sample_report(current_time_step_sample_,
                            trial_controller_->get_trial_length());
      trial_controller_->reset(current_time);
      trial_controller_.reset(NULL); // WTF is this!?

      // Set active arm controller to NO_CONTROL
      OptionsMap options;
      options["mode"] = gps::NO_CONTROL;
      active_arm_controller_->configure_controller(options);

      // At this point in the RobotPlugin code, there is an empty loop to switch
      // on full-speed sensing at a later date. Mind this.

    }

  if (active_arm_controller_->report_waiting &&
      active_arm_controller_->is_finished())
    {
      publish_sample_report(current_time_step_sample_);
      active_arm_controller_->report_waiting = false;
    }

  if (passive_arm_controller_->report_waiting &&
      passive_arm_controller_->is_finished())
    passive_arm_controller_->report_waiting = false;
}


void SingleArmPlugin::position_subscriber_callback(const
    gps_agent_pkg::PositionCommand::ConstPtr& msg)
{
  ROS_INFO_STREAM("received position command");
  OptionsMap params;
  int8_t arm = msg->arm;
  params["mode"] = msg->mode;
  Eigen::VectorXd data;
  data.resize(msg->data.size());
  for (int i = 0; i < data.size(); i++)
    {
      data[i] = msg->data[i];
    }
  params["data"] = data;

  Eigen::MatrixXd pd_gains;
  pd_gains.resize(msg->pd_gains.size() / 4, 4);
  for (int i = 0; i < pd_gains.rows(); i++)
    {
      for (int j = 0; j < 4; j++)
        {
          pd_gains(i, j) = msg->pd_gains[i * 4 + j];
        }
    }
  params["pd_gains"] = pd_gains;

  if (arm == gps::TRIAL_ARM)
    {
      active_arm_controller_->configure_controller(params);
    }
  else if (arm == gps::AUXILIARY_ARM)
    {
      ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT <<
                      "Passive arm position command received.");
    }
  else
    {
      ROS_ERROR("Unknown position controller arm type");
    }
}


Sensor*
SingleArmPlugin::get_sensor(SensorType sensor, gps::ActuatorType actuator_type)
{
  if (actuator_type == gps::TRIAL_ARM)
    {
      assert(sensor < TotalSensorTypes);
      return sensors_[sensor].get();
    }
  else if (actuator_type == gps::AUXILIARY_ARM)
    {
      ROS_INFO_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT <<
                      "Getting from absent passive arm");
    }
}



void SingleArmPlugin::relax_subscriber_callback(const
    gps_agent_pkg::RelaxCommand::ConstPtr& msg)
{
  ROS_INFO_STREAM("received relax command");
  OptionsMap params;
  int8_t arm = msg->arm;
  params["mode"] = gps::NO_CONTROL;

  if (arm == gps::TRIAL_ARM)
    {
      active_arm_controller_->configure_controller(params);
    }
  else if (arm == gps::AUXILIARY_ARM)
    {
      ROS_ERROR_STREAM(SINGLE_ARM_CLASS_NAME_TO_PRNT <<
                       "Ordered passive arm to relax");
    }
  else
    {
      ROS_ERROR("Unknown position controller arm type");
    }
}


void SingleArmPlugin::initialize_position_controllers(ros::NodeHandle& n)
{
  // Create passive arm position controller.
  // TODO: fix this to be something that comes out of the robot itself
  passive_arm_controller_.
    reset(new PositionController(n, gps::AUXILIARY_ARM, 7));

  // Create active arm position controller.
  active_arm_controller_.
    reset(new PositionControllerLWRHack(n, gps::TRIAL_ARM, 7));
}