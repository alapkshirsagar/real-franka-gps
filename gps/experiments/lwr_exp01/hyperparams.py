#  _  ___   _ _  __   _     _ __      _____   _ _    _   
# | |/ / | | | |/ /  /_\   | |\ \    / / _ \ | | | _| |_ 
# | ' <| |_| | ' <  / _ \  | |_\ \/\/ /|   / |_  _|_   _|
# |_|\_\\___/|_|\_\/_/ \_\ |____\_/\_/ |_|_\   |_|  |_|  

# Hyperparameters for KUKA LWR 4+ demonstration lwr_exp01
# Visit rll.berkeley.edu/gps/hyperparams.html for documentation.

from __future__ import division # Fix inconsistent Python 2.7 division op
from datetime import datetime
import os.path
import numpy as np
from gps import __file__ as gps_filepath
from gps.agent.ros_alt.agent_ros_alt import AgentROSAlt
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info
from gps.gui.util import load_pose_from_npz

# Experiment name
EXP_NAME = 'lwr_exp01'

# Explicitly use only the end of the KUKA arm
EE_POINTS = np.array([[0.0, 0.0, 0.0]])

# Specifications for subsequent array lengths
SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 7
}


# Default gains---actually set to 1 by LQR
DEF_GAINS = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# Used in logging and target config
BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/' + EXP_NAME + '/'

# Setup for logging and target config
common = {
    'experiment_name': EXP_NAME + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 1,
}

# Possibly not needed
if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

# Conditions setup
x0s = [] # Internal initial state
ee_tgts = [] # Internal end-effector target
reset_conditions = [] # Initial joint position per arm

for i in range(common['conditions']): # xrange replaced for Python 3 future
    # Load initial joint angles and end-ffector position and rotation
    ja_x0, ee_pos_x0, ee_rot_x0 = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'initial')
    _, ee_pos_tgt, ee_rot_tgt = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'target'
    )

    # Allocate for initial state in numpy format
    x0 = np.zeros(SENSOR_DIMS[JOINT_ANGLES] + SENSOR_DIMS[JOINT_VELOCITIES] + \
                  SENSOR_DIMS[END_EFFECTOR_POINTS] + \
                  SENSOR_DIMS[END_EFFECTOR_POINT_VELOCITIES])

    # Set initial joint angles (from NPZ)
    x0[:7] = ja_x0
    x0[14:(14 + 3 * EE_POINTS.shape[0])] = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_x0, ee_rot_x0).T
    )

    # Set target position
    ee_tgt = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T
    )

    # Set up return to initial position
    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[0:7]
        }
    }

    # Add (single) condition to 'list' of conditions
    # x0 should now contain initial joint angles, initial-zero joint velocities,
    # initial end-effector point and initial-zero end-effector joint velocities
    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)

# agent['obs_include'] is not used by AgentROSAlt, but appears in the
# constructor for Agent
agent = {
    'type': AgentROSAlt, # Agent for ROS with ros_control
    'dt': 0.05, # Step time
    'conditions': common['conditions'], # Experiment conditions
    'T': 100, # Number of steps
    'x0': x0s, # Initial state
    'ee_points_tgt': ee_tgts, # Target end-effector point
    'reset_conditions': reset_conditions, # Position @ t=0
    'sensor_dims': SENSOR_DIMS, # Dimensions of sensors
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [],
}

algorithm = {
    'type': AlgorithmTrajOpt,
    'conditions': common['conditions'],
    'iterations': 10,
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / DEF_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1.0,
    'stiffness': 0.5,
    'stiffness_vel': 0.25,
    'final_weight': 50,
    'dt': agent['dt'],
    'T': agent['T'],
}

torque_cost = {
    'type': CostAction,
    'wu': 5e-3 / DEF_GAINS,
}

fk_cost1 = {
    'type': CostFK,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 0.1,
    'l2': 0.0001,
    'ramp_option': RAMP_LINEAR,
}

fk_cost2 = {
    'type': CostFK,
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 1.0,
    'l2': 0.0,
    'wp_final_multiplier': 10.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [torque_cost, fk_cost1, fk_cost2],
    'weights': [1.0, 1.0, 1.0],
}

algorithm['dynamics'] = {
    'type': DynamicsLRPrior,
    'regularization': 1e-6,
    'prior': {
        'type': DynamicsPriorGMM,
        'max_clusters': 20,
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    },
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {}

config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 0,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
    'num_samples': 5,
}

common['info'] = generate_experiment_info(config)