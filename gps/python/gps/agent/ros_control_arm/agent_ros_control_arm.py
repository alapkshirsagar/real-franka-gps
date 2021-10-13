""" This file defines an agent for a single-arm ROS controller. """
import copy # For deep copying the default configuration
import time
import numpy as np # Following convention of agent_ros

import rospy
import tf
from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS

# Continue to import utils from the PR2 agent - no code is changed
from gps.agent.ros_control_arm.agent_ros_control_arm_utils import \
    ServiceEmulator, msg_to_sample, policy_to_msg, tf_policy_to_action_msg, \
    tf_obs_msg_to_numpy

# Change from agent_ros is to only import TRIAL_ARM
from gps.proto.gps_pb2 import TRIAL_ARM
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
                              RelaxCommand, DataRequest, TfActionCommand, \
                              TfObsData

try: # i.e. if Tensorflow is present
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:
    TfPolicy = None


class AgentROSControlArm(Agent):
    """
    Facilitates communication between GPS algorithms and the single-arm
    ROS controller for GPS.
    """

    def __init__(self, hyperparams, init_node=True):
        """
        Initialise agent.
        Args:
            hyperparams: Dictionary of hyperparameters.
            init_node: Whether to initialise a new ROS node.
        """
        config = copy.deepcopy(AGENT_ROS)

        config['pid_params'] = 10.0*np.array([ \
            0.6, 0.3, 0.3, 0.1, \
            0.6, 0.3, 0.3, 0.1, \
            0.6, 0.3, 0.3, 0.1, \
            0.6, 0.3, 0.3, 0.1, \
            0.25, 0.1, 0.1, 0.1, \
            0.25, 0.1, 0.1, 0.1, \
            0.25, 0.1, 0.1, 0.1])

        config.update(hyperparams)
        Agent.__init__(self, config)
        if init_node:
            rospy.init_node('gps_agent_ros_control_arm_node');
        self._init_pubs_and_subs()
        self._seq_id = 0 # Used for setting seq in ROS commands.

        conditions = self._hyperparams['conditions'] # Target pose

        self.x0 = []
        self.tf_listener = tf.TransformListener()
        pose_available = False
        r = rospy.Rate(20) # 100hz

        while not pose_available:
            try:
                (trans_robot, rot_robot) = self.tf_listener.lookupTransform("optitrack_origin", "kinova_gripper", rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                (trans_human, rot_human) = self.tf_listener.lookupTransform("optitrack_origin", "human_hand", rospy.Time(0))
                # print("Human position =", trans_human)
                # print("Robot position =", trans_robot)
                pose_available = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
            r.sleep()

        eepts_notgt = np.concatenate([trans_human, trans_human])
        # if self._hyperparams['target_end_effector'] == 'human_hand':
        #     for field in ['x0']:
        #         self._hyperparams[field] = setup(self._hyperparams[field], \
        #                                         conditions)
        # else:
        for field in ('x0', 'ee_points_tgt', 'reset_conditions'):
            self._hyperparams[field] = setup(self._hyperparams[field], \
                                            conditions)
        if len(self._hyperparams['x0'][0]) != 32:
            # import pdb; pdb.set_trace()
            self.x0.append(
                        np.concatenate([self._hyperparams['x0'][0], eepts_notgt, np.zeros_like(eepts_notgt)])
                    )
        else:
            self.x0 = self._hyperparams['x0']

        self.use_tf = False
        self.observations_stale = True

    def _init_pubs_and_subs(self):
        """
        Initialise publication and subscription for the GPS command
        pseudo-services.
        """
        self._trial_service = ServiceEmulator(
            self._hyperparams['trial_command_topic'], TrialCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )

        self._reset_service = ServiceEmulator(
            self._hyperparams['reset_command_topic'], PositionCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )

        self._relax_service = ServiceEmulator(
            self._hyperparams['relax_command_topic'], RelaxCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )

        self._data_service = ServiceEmulator(
            self._hyperparams['data_request_topic'], DataRequest,
            self._hyperparams['sample_result_topic'], SampleResult
        )

    def _get_next_seq_id(self):
        """
        Generate a new sequence ID for messages.
        """
        self._seq_id = (self._seq_id + 1) % (2 ** 32)
        return self._seq_id

    def get_data(self, arm=TRIAL_ARM):
        """
        Request for the most recent sensor readings.
        Returns entire sample report.
        Args:
            arm: TRIAL_ARM or AUXILIARY_ARM. Aux arm requests are exceptional.
        """
        if arm != TRIAL_ARM:
            raise NotImplementedError('Only trial arm is accepted')
        request = DataRequest()
        request.id = self._get_next_seq_id()
        request.arm = TRIAL_ARM
        request.stamp = rospy.get_rostime()
        result_msg = self._data_service.publish_and_wait(request)
        sample = msg_to_sample(result_msg, self)
        return sample

    def relax_arm(self, arm):
        """
        Relax the arm of the robot.
        Args:
            arm: TRIAL_ARM or AUXILIARY_ARM. Aux arm requests are exceptional.
        """
        if arm != TRIAL_ARM:
            raise NotImplementedError('Only trial arm is accepted')

        relax_command = RelaxCommand()
        relax_command.id = self._get_next_seq_id()
        relax_command.arm = TRIAL_ARM
        relax_command.stamp = rospy.get_rostime()
        self._relax_service.publish_and_wait(relax_command)

    def reset_arm(self, arm, mode, data):
        """
        Reset the robot arm by position command.
        Args:
            arm: TRIAL_ARM or AUXILIARY_ARM. Aux arm requests are exceptional.
            mode: An integer code defined in gps_pb2
            data: An array of floats
        """
        if arm != TRIAL_ARM:
            raise NotImplementedError('Only trial arm is accepted')

        reset_command = PositionCommand()
        reset_command.mode = mode
        reset_command.data = data
        reset_command.pd_gains = self._hyperparams['pid_params']
        reset_command.arm = TRIAL_ARM
        timeout = self._hyperparams['trial_timeout']
        reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(reset_command)
        rospy.sleep(5.0)

    def reset(self, condition):
        """
        Reset the agent for a particular experiment condition.
        Args:
            condition: An index into hyperparams['reset_conditions'].
        """
        condition_data = self._hyperparams['reset_conditions'][condition]
        self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'], \
                       condition_data[TRIAL_ARM]['data'])
        time.sleep(2.0) # Allows a physical robot to come to a full stop.

    def sample(self, policy, condition, iteration, verbose=True, save=True, noisy=True):
        """
        Reset and execute a policy and collect a (compound) sample.
        Args:
            policy: A Policy object.
            condition: Which condition to run towards.
            verbose: Unused (as per agent_ros).
            save: Whether or not to save the trial into the samples.
            noisy: Whether or not to use noise during sampling.
        Returns:
            sample: A Sample object.
        """
        if TfPolicy is not None:  # user has tf installed.
            if isinstance(policy, TfPolicy):
                self._init_tf(policy.dU)

        self.reset(condition)
        # Generate noise.
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))

        # Execute trial.
        trial_command = TrialCommand()
        trial_command.id = self._get_next_seq_id()
        trial_command.controller = policy_to_msg(policy, noise)
        trial_command.T = self.T
        trial_command.id = self._get_next_seq_id()
        trial_command.frequency = self._hyperparams['frequency']
        ee_points = self._hyperparams['end_effector_points']
        trial_command.ee_points = ee_points.reshape(ee_points.size).tolist()
        trial_command.ee_points_tgt = \
                self._hyperparams['ee_points_tgt'][condition].tolist()
        trial_command.state_datatypes = self._hyperparams['state_include']
        trial_command.obs_datatypes = self._hyperparams['state_include']
        # import pdb; pdb.set_trace()
        if self.use_tf is False:
            sample_msg = self._trial_service.publish_and_wait(
                trial_command, timeout=self._hyperparams['trial_timeout']
            )
            sample = msg_to_sample(sample_msg, self)
            if save:
                self._samples[condition].append(sample)
            return sample
        else:
            self._trial_service.publish(trial_command)
            sample_msg = self.run_trial_tf(policy, \
                time_to_run = self._hyperparams['trial_timeout'])
            sample = msg_to_sample(sample_msg, self)
            if save:
                self._samples[condition].append(sample)
            return sample

    def run_trial_tf(self, policy, time_to_run=5):
        """
        Run an async controller from a policy. The async controller receives
        observations from ROS subscribers and then uses them to publish actions.
        """
        should_stop = False
        consecutive_failures = 0
        start_time = time.time()
        while should_stop is False:
            if self.observations_stale is False:
                consecutive_failures = 0
                last_obs = tf_obs_msg_to_numpy(self._tf_subscriber_msg)
                action_msg = \
                    tf_policy_to_action_msg(self.dU,
                                            self._get_new_action(policy, \
                                                                 last_obs), \
                                            self.current_action_id)
                self._tf_publish(action_msg)
                self.observations_stale = True
                self.current_action_id += 1
            else:
                rospy.sleep(0.01)
                consecutive_failures += 1
                if time.time() - start_time > time_to_run \
                   and consecutive_failures > 5:
                    # we only stop when we have run for the trial time and are
                    # no longer receiving obs.
                    should_stop = True
        rospy.sleep(0.25)  # wait for finished trial to come in.
        result = self._trial_service._subscriber_msg
        return result  # the trial has completed. Here is its message.

    def _get_new_action(self, policy, obs):
        return policy.act(None, obs, None, None)

    def _tf_callback(self, message):
        self._tf_subscriber_msg = message
        self.observations_stale = False

    def _tf_publish(self, pub_msg):
        """ Publish a message without waiting for response. """
        self._pub.publish(pub_msg)

    def _init_tf(self, dU):
        self._tf_subscriber_msg = None
        self.observations_stale = True
        self.current_action_id = 1
        self.dU = dU
        if self.use_tf is False:  # init pub and sub if this not called before.
            self._pub = \
                rospy.Publisher('/gps_controller_sent_robot_action_tf', \
                                TfActionCommand)
            self._sub = \
                rospy.Subscriber('/gps_obs_tf', TfObsData, self._tf_callback)
            r = rospy.Rate(0.5)  # wait for publisher/subscriber to kick on.
            r.sleep()
        self.use_tf = True
        self.observations_stale = True
