/*****************************************************************************
 * KUKA LWR 4+ ROS ros_control plugin for GPS
 * Jack White, Aalto University 2018
 *****************************************************************************/

#include "gps_agent_pkg/lwrplugin.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/trialcontroller.h"
#include "gps_agent_pkg/encodersensor.h"
#include "gps_agent_pkg/util.h"

#include <string>
#include <pluginlib/class_list_macros.h>

namespace gps_control
{

KUKALWRPlugin::KUKALWRPlugin()
{
  controller_counter_ = 0;
  controller_step_len_ms_ = 50;
}

KUKALWRPlugin::~KUKALWRPlugin()
{
}

bool
KUKALWRPlugin::init(hw_ifc::EffortJointInterface* robot, ros::NodeHandle& n)
{
  std::string robot_description; // URDF description
  std::string root_name;
  std::string tip_name;

  robot_ = robot;

  // ============================================== STEP: Get root and tip names
  if (!n.getParam("root_name", root_name))
    {
      ROS_ERROR("KUKALWRPlugin: Property root_name not found in namespace: '%s'",
                n.getNamespace().c_str());
      return false;
    }
  if (!n.getParam("tip_name", tip_name))
    {
      ROS_ERROR("KUKALWRPlugin: Property tip_name not found in namespace: '%s'",
                n.getNamespace().c_str());
      return false;
    }

  // ================================== STEP: Find description in parameter tree
  if (!ros::param::search(n.getNamespace(), "robot_description",
                          robot_description))
    {
      ROS_ERROR_STREAM("KUKALWRPlugin: No robot description (URDF) found on parameter server ("
                       << n.getNamespace() << "/robot_description)");
      return false;
    }

  // =========================== STEP: Get description and convert to URDF model
  std::string xml_description;

  if (n.hasParam(robot_description))
    n.getParam(robot_description, xml_description);
  else
    {
      ROS_ERROR("KUKALWRPlugin: Parameter %s not set, shutting down node...",
                robot_description.c_str());
      n.shutdown();
      return false;
    }

  if (xml_description.size() == 0)
    {
      ROS_ERROR("KUKALWRPlugin: Unable to load robot model from parameter %s",
                robot_description.c_str());
      n.shutdown();
      return false;
    }

  ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_description.c_str());

  urdf::Model model;
  if (!model.initString(xml_description))
    {
      ROS_ERROR("Failed to parse urdf file");
      n.shutdown();
      return false;
    }
  ROS_INFO("Successfully parsed urdf file");

  KDL::Tree kdl_tree_;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
      ROS_ERROR("Failed to construct kdl tree");
      n.shutdown();
      return false;
    }

  // ============================================== STEP: Populate the KDL chain
  if (!kdl_tree_.getChain(root_name, tip_name, active_arm_fk_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
      ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
      ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
      // ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree_.getSegments();
      KDL::SegmentMap::iterator it;

      for ( it = segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    " << (*it).first);

      return false;
    }

  ROS_DEBUG("Number of segments: %d", active_arm_fk_chain_.getNrOfSegments());
  ROS_DEBUG("Number of joints in chain: %d",
            active_arm_fk_chain_.getNrOfJoints());


  // ---------------------------------------- Continuing with PR2 plugin gubbins
  // ==========STEP: Set up solvers for chain forward kinematics and EE Jacobian
  active_arm_fk_solver_.
    reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));
  active_arm_jac_solver_.
    reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));


  // =================================================== STEP: Get joint handles
  // ================================= takes the place of the direct acquisition
  // ====================================== of the joint names in the PR2 plugin
  for (std::vector<KDL::Segment>::const_iterator it =
         active_arm_fk_chain_.segments.begin();
       it != active_arm_fk_chain_.segments.end();
       ++it)
    {
      if (it->getJoint().getType() != KDL::Joint::None)
        {
          joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
        }
    }

  ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size() );

  active_arm_torques_.resize(active_arm_fk_chain_.getNrOfJoints());

  initialize(n); // RobotPlugin initialisation

  return true;
}

void
KUKALWRPlugin::starting(const ros::Time& time)
{
  ROS_DEBUG("Entering starting()");
  // From the PR2 plugin
  last_update_time_ = time;
  controller_counter_ = 0;
  sensors_[0]->reset(this, last_update_time_); // This is done once per type of
                                               // sensor - what is a sensor?
  active_arm_controller_->reset(last_update_time_);
  if (trial_controller_ != NULL)
    trial_controller_->reset(last_update_time_);

  // NB. LWR controllers get the joint state here. PR2 waits until update
  ROS_DEBUG("Exiting starting()");
}


void
KUKALWRPlugin::stopping(const ros::Time& time)
{
  // Empty by design
  ROS_DEBUG("Entering stopping()");
  ROS_DEBUG("Exiting stopping()");
}

void
KUKALWRPlugin::update(const ros::Time& time, const ros::Duration& period)
{
  ROS_DEBUG("Entering update()");
  last_update_time_ = time;

  // Check if this is a controller step
  controller_counter_++;
  if (controller_counter_ >= controller_step_len_ms_)
    controller_counter_ = 0;

  // Update GPS sensors and controllers
  ROS_DEBUG("  Updating sensors...");
  update_sensors(last_update_time_, controller_counter_ == 0);
  ROS_DEBUG("  Updating controllers...");
  update_controllers(last_update_time_, controller_counter_ == 0);

  ROS_DEBUG("  Storing torques...");
  // Store the torques

  std::cout << active_arm_torques_.transpose() << "\n";
  for (uint8_t i(0); i < joint_handles_.size(); i++)
    {
      joint_handles_[i].setCommand(active_arm_torques_(i));
    }

  ROS_DEBUG("Exiting update()");
}

void
KUKALWRPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles,
    gps::ActuatorType arm) const
{
  ROS_DEBUG("      Entering get_joint_encoder_readings");
  switch (arm)
    {
    case gps::TRIAL_ARM:
      ROS_DEBUG("        Getting joint readings for trial arm");

      if (angles.rows() != joint_handles_.size())
        angles.resize(joint_handles_.size());

      for (uint8_t i(0); i < angles.size(); i++)
        angles(i) = joint_handles_[i].getPosition();
      
      break;
    case gps::AUXILIARY_ARM:
      ROS_DEBUG("        KUKALWRPlugin requested use of passive (missing) arm");
      break;
    default:
      ROS_ERROR("       KUKALWRPlugin: Invalid arm requested for encoder readings");
      break;
    }
  ROS_DEBUG("        Exiting get_joint_encoder_readings");
}


ros::Time
KUKALWRPlugin::get_current_time() const
{
  return last_update_time_;
}

}

/* This follows a newer format for registering plugins, used in the KUKA
   controller code. The PR2 plugin uses PLUGINLIB_DECLARE_CLASS, but this is
   deprecated. */
PLUGINLIB_EXPORT_CLASS( gps_control::KUKALWRPlugin,
                        controller_interface::ControllerBase )
