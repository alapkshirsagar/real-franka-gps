#include "gps_agent_pkg/encodersensor.h"
#include "gps_agent_pkg/robotplugin.h"

using namespace gps_control;

// Constructor.
EncoderSensor::EncoderSensor(ros::NodeHandle& n, RobotPlugin *plugin, gps::ActuatorType actuator_type): Sensor(n, plugin)
{
    // Set internal arm
    actuator_type_ = actuator_type;

    // Get current joint angles.
    plugin->get_joint_encoder_readings(previous_angles_, actuator_type);

    // Initialize velocities.
    previous_velocities_.resize(previous_angles_.size());

    // Initialize temporary angles.
    temp_joint_angles_.resize(previous_angles_.size());

    // Resize KDL joint array.
    temp_joint_array_.resize(previous_angles_.size());


    // Set time.
    previous_angles_time_ = ros::Time(0.0); // This ignores the velocities on the first step.

    // Initialize and configure Kalman filter
    joint_filter_.reset(new EncoderFilter(n, previous_angles_));
}

// Destructor.
EncoderSensor::~EncoderSensor()
{
    // Nothing to do here.
}

// Update the sensor (called every tick).
void EncoderSensor::update(RobotPlugin *plugin, ros::Time current_time, bool is_controller_step)
{
    ROS_DEBUG("      Entering EncoderSensor::update()");
    double update_time = current_time.toSec() - previous_angles_time_.toSec();

    // Get new vector of joint angles from plugin.
    plugin->get_joint_encoder_readings(temp_joint_angles_, actuator_type_);
    joint_filter_->update(update_time, temp_joint_angles_);

    if (is_controller_step)
    {
        // Get filtered joint angles
        joint_filter_->get_state(temp_joint_angles_);

        // IMPORTANT: note that the Python code will assume that the Jacobian is the Jacobian of the end effector points, not of the end
        // effector itself. In the old code, this correction was done in Matlab, but since the simulator will produce Jacobians of end
        // effector points directly, it would make sense to also do this transformation on the robot, and send back N Jacobians, one for
        // each feature point.


        // Compute velocities.
        // Note that we can't assume the last angles are actually from one step ago, so we check first.
        // If they are roughly from one step ago, assume the step is correct, otherwise use actual time.

        double update_time = current_time.toSec() - previous_angles_time_.toSec();
        if (!previous_angles_time_.isZero())
        { // Only compute velocities if we have a previous sample.
            if (fabs(update_time)/sensor_step_length_ >= 0.5 &&
                fabs(update_time)/sensor_step_length_ <= 2.0)
            {
                previous_end_effector_point_velocities_ = (temp_end_effector_points_ - previous_end_effector_points_)/sensor_step_length_;
                for (unsigned i = 0; i < previous_velocities_.size(); i++){
                    previous_velocities_[i] = (temp_joint_angles_[i] - previous_angles_[i])/sensor_step_length_;
                }
            }
            else
            {
                previous_end_effector_point_velocities_ = (temp_end_effector_points_ - previous_end_effector_points_)/update_time;
                for (unsigned i = 0; i < previous_velocities_.size(); i++){
                    previous_velocities_[i] = (temp_joint_angles_[i] - previous_angles_[i])/update_time;
                }
            }
        }

        // Move temporaries into the previous joint angles.
        for (unsigned i = 0; i < previous_angles_.size(); i++){
            previous_angles_[i] = temp_joint_angles_[i];
        }

        // Update stored time.
        previous_angles_time_ = current_time;
    }
    ROS_DEBUG("      Exiting EncoderSensor::update()");
}

void EncoderSensor::configure_sensor(OptionsMap &options)
{
    /* TODO: note that this will get called every time there is a report, so
    we should not throw out the previous transform just because we are trying
    to set end-effector points. Instead, just use the stored transform to
    compute what the points should be! This will allow us to query positions
    and velocities each time. */
    ROS_DEBUG("Nothing to do here");

}

// Set data format and meta data on the provided sample.
void EncoderSensor::set_sample_data_format(boost::scoped_ptr<Sample>& sample)
{
    // Set joint angles size and format.
    OptionsMap joints_metadata;
    sample->set_meta_data(gps::JOINT_ANGLES,previous_angles_.size(),SampleDataFormatEigenVector,joints_metadata);

    // Set joint velocities size and format.
    OptionsMap velocities_metadata;
    sample->set_meta_data(gps::JOINT_VELOCITIES,previous_velocities_.size(),SampleDataFormatEigenVector,joints_metadata);
}

// Set data on the provided sample.
void EncoderSensor::set_sample_data(boost::scoped_ptr<Sample>& sample, int t)
{
    // Set joint angles.
    sample->set_data_vector(t,gps::JOINT_ANGLES,previous_angles_.data(),previous_angles_.size(),SampleDataFormatEigenVector);

    // Set joint velocities.
    sample->set_data_vector(t,gps::JOINT_VELOCITIES,previous_velocities_.data(),previous_velocities_.size(),SampleDataFormatEigenVector);

}
