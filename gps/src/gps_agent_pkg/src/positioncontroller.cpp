#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/util.h"

#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace gps_control;

// Constructor.

// Constructor.
PositionController::PositionController(ros::NodeHandle& n, gps::ActuatorType arm, int size)
    : Controller(n, arm, size)
{
    // Initialize PD gains.
    pd_gains_p_.resize(size);
    pd_gains_d_.resize(size);
    pd_gains_i_.resize(size);

    // Initialize velocity bounds.
    max_velocities_.resize(size);

    // Initialize integral terms to zero.
    pd_integral_.resize(size);
    i_clamp_.resize(size);

    // Initialize current angle and position.
    current_angles_.resize(size);
    current_angle_velocities_.resize(size);
    current_pose_.resize(size);

    // Initialize target angle and position.
    target_angles_.resize(size);
    target_pose_.resize(size);

    // Initialize joints temporary storage.
    temp_angles_.resize(size);

    // Initialize Jacobian temporary storage.
    temp_jacobian_.resize(6,size);

    // Set initial mode.
    mode_ = gps::NO_CONTROL;

    // Set initial time.
    last_update_time_ = ros::Time(0.0);

    // Set arm.
    arm_ = arm;

    //
    report_waiting = false;
}

// Destructor.
PositionController::~PositionController()
{
}

// Update the controller (take an action).
void PositionController::update(RobotPlugin *plugin, ros::Time current_time, boost::scoped_ptr<Sample>& sample, Eigen::VectorXd &torques)
{
    //ROS_INFO_STREAM("1");
    // Get current joint angles.
    plugin->get_joint_encoder_readings(temp_angles_, arm_);

    // Check dimensionality.
    assert(temp_angles_.rows() == torques.rows());
    assert(temp_angles_.rows() == current_angles_.rows());
  
    //ROS_INFO_STREAM("2");

    // Estimate joint angle velocities.
    double update_time = current_time.toSec() - last_update_time_.toSec();
    if (!last_update_time_.isZero())
    { // Only compute velocities if we have a previous sample.
        current_angle_velocities_ = (temp_angles_ - current_angles_)/update_time;
    }
    
    //ROS_INFO_STREAM("3");

    // Store new angles.
    current_angles_ = temp_angles_;

    // Update last update time.
    last_update_time_ = current_time;
    
    //ROS_INFO_STREAM("4");

    // If doing task space control, compute joint positions target.
    if (mode_ == gps::TASK_SPACE)
    {
        ROS_ERROR("Not implemented!");

        // TODO: implement.
        // Get current end effector position.

        // Get current Jacobian.

        // TODO: should also try Jacobian pseudoinverse, it may work a little better.
        // Compute desired joint angle offset using Jacobian transpose method.
        target_angles_ = current_angles_ + temp_jacobian_.transpose() * (target_pose_ - current_pose_);
    }

    //ROS_INFO_STREAM("5");


    // If we're doing any kind of control at all, compute torques now.
    if (mode_ != gps::NO_CONTROL)
    {
        //std::cout << "Position Controller Active" << "\n";

        // Compute error.
        //ROS_INFO_STREAM("Cur angs: " << current_angles_ <<
        //               "Tar angs: " << target_angles_);
        temp_angles_ = current_angles_ - target_angles_;

        // Add to integral term.
        pd_integral_ += temp_angles_ * update_time;

        //std::cout << "Position Controller Active 2" << "\n";

        // Clamp integral term
        for (int i = 0; i < temp_angles_.rows(); i++){
            if (pd_integral_(i) * pd_gains_i_(i) > i_clamp_(i)) {
                pd_integral_(i) = i_clamp_(i) / pd_gains_i_(i);
            }
            else if (pd_integral_(i) * pd_gains_i_(i) < -i_clamp_(i)) {
                pd_integral_(i) = -i_clamp_(i) / pd_gains_i_(i);
            }
        }
    
    //ROS_INFO_STREAM("6");

        // Compute torques.
        // ROS_DEBUG_STREAM("P: " << (pd_gains_p_.array() *
        //                            temp_angles_.array()).transpose());
        // ROS_DEBUG_STREAM("D: " << (pd_gains_d_.array() *
        //                            current_angle_velocities_.array()).transpose());
        // ROS_DEBUG_STREAM("I: " << (pd_gains_i_.array() *
        //                            pd_integral_.array()).transpose());

        torques = -((pd_gains_p_.array() * temp_angles_.array()) +
                    (pd_gains_d_.array() * current_angle_velocities_.array()) +
                    (pd_gains_i_.array() * pd_integral_.array())).matrix();
        //std::cout << "Current Angles: " << current_angles_.transpose() << "\n";
        //std::cout << "Target Angles: " << target_angles_.transpose() << "\n";
        // std::cout << "Torques: " << torques.transpose() << "\n";
        // std::cout << "P_gains: " << (pd_gains_p_.array()).transpose() << "\n";
        // std::cout << "Temp angles: " << (temp_angles_.array()).transpose() << "\n";
        // std::cout << "P: " << (pd_gains_p_.array() *
        //                            temp_angles_.array()).transpose() << "\n";
        // std::cout << "D: " << (pd_gains_d_.array() *
        //                            current_angle_velocities_.array()).transpose() << "\n";
        // std::cout << "I: " << (pd_gains_i_.array() *
        //                            pd_integral_.array()).transpose() << "\n";
        // std::this_thread::sleep_for (std::chrono::seconds(1));

    }
    else
    {
        torques = Eigen::VectorXd::Zero(torques.rows());
        //ROS_INFO_STREAM("7");
    }

}

// Configure the controller.
void PositionController::configure_controller(OptionsMap &options)
{
    // This sets the target position.
    // This sets the mode
    ROS_INFO_STREAM("Received controller configuration");
    // needs to report when finished
    report_waiting = true;
    mode_ = (gps::PositionControlMode) boost::get<int>(options["mode"]);
    if (mode_ != gps::NO_CONTROL){
        Eigen::VectorXd data = boost::get<Eigen::VectorXd>(options["data"]);
        Eigen::MatrixXd pd_gains = boost::get<Eigen::MatrixXd>(options["pd_gains"]);
        for(int i=0; i<pd_gains.rows(); i++){
            pd_gains_p_(i) = pd_gains(i, 0);
            pd_gains_i_(i) = pd_gains(i, 1);
            pd_gains_d_(i) = pd_gains(i, 2);
            i_clamp_(i) = pd_gains(i, 3);
        }
        if(mode_ == gps::JOINT_SPACE){
            target_angles_ = data;
        }else{
            ROS_ERROR("Unimplemented position control mode!");
        }
    }
}

// Check if controller is finished with its current task.
bool PositionController::is_finished() const
{
    // Check whether we are close enough to the current target.
    if (mode_ == gps::JOINT_SPACE){
        // double epspos = 0.185;
        // double epsvel = 0.01;
        double epspos = 0.385;
        double epsvel = 0.03;
        double error = (current_angles_ - target_angles_).norm();
        double vel = current_angle_velocities_.norm();
        // ROS_INFO_STREAM("PosCn finished? poserr: " << error << " vel: " <<
        //                 vel << " T/H: " << epspos << "/" << epsvel);
        return (error < epspos && vel < epsvel);
    }
    else if (mode_ == gps::NO_CONTROL){
        return true;
    }
}

// Reset the controller -- this is typically called when the controller is turned on.
void PositionController::reset(ros::Time time)
{
    // Clear the integral term.
    pd_integral_.fill(0.0);

    // Clear update time.
    last_update_time_ = ros::Time(0.0);
}

