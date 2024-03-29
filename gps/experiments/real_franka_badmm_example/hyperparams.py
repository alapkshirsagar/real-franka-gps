# To get started, copy over hyperparams from another experiment.
# Visit rll.berkeley.edu/gps/hyperparams.html for documentation.

""" Hyperparameters for Franka-Emika Panda trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros_control_arm.agent_ros_control_arm import AgentROSControlArm
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.gui.target_setup_gui import load_pose_from_npz
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info
from gps.algorithm.policy_opt.policy_opt_caffe import PolicyOptCaffe
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM




# EE_POINTS = np.array([[0.22, -0.025, 0.55], [0.22, -0.025, -0.55],
#                       [0.22, 0.05, 0.5]])
# EE_POINTS = np.array([[0.22, -0.025, 0.55], [0.22, -0.025, -0.55]])
# EE_POINTS = np.array([[0.22, 0.22], [-0.025, -0.025], [0.55, 0.55]])
# EE_POINTS = np.array([[-0.07529071044921876, 0.06009528413414955, 0.012681339383125295], [-0.07529071044921876, 0.06009528413414955, 0.012681339383125295]])
# EE_POINTS = np.array([[-0.07529071044921876, 0.06009528413414955, 0.012681339383125295], [-0.07529071044921876, 0.06009528413414955, 0.012681339383125295], [-0.07529071044921876, 0.06009528413414955, 0.012681339383125295]])
EE_POINTS = np.array([[-0.07529071044921876, 0.06009528413414955, 0.012681339383125295], [-0.07529071044921876, 0.06009528413414955, 0.012681339383125295]]) #for relative


SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 6,
    END_EFFECTOR_POINT_VELOCITIES: 6,
    ACTION: 7,
}

# PR2_GAINS = np.array([]) #
#PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])
FRANKA_GAINS = np.array([0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001]) # [24, 12, 10, 7, 3, 3, 6]


BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/real_franka_badmm_example/'
# EXP_DIR = '/media/franka2/DATA/Kuka_Franka_Experiment_Data/real_franka_badmm_example/'

x0s = []
ee_tgts = []
reset_conditions = []

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 8,
}

# TODO(chelsea/zoe) : Move this code to a utility function
# Set up each condition.
for i in xrange(common['conditions']):

    ja_x0, ee_pos_x0, ee_rot_x0 = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'initial'
    )
    _, ee_pos_tgt, ee_rot_tgt = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'target'
    )

    x0 = np.zeros(7+7+6*EE_POINTS.shape[0])
    # x0[:7] = [0,0.5,0,-0.5,0, 0.5,0]
    # x0[:7] = [-0.000159,-0.783775,0.000139,-2.356250,0.000304,1.570931,0.784746]
    x0[:7] = [-0.06858-0.1,0.70147,0.08587-0.1,-2.119857,-0.049951,2.9128,0.7915173]
    # x0[:7] = [0.1, 0.8, 0.1, -2, -0.1, 3, 0.9]
    # x0[14:(14+3*EE_POINTS.shape[0])] = np.ndarray.flatten(
    #     get_ee_points(EE_POINTS, ee_pos_x0, ee_rot_x0).T
    # )

    # ee_tgt = np.ndarray.flatten(
    #     get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T
    # )

    x0[14:(14+3*EE_POINTS.shape[0])] = EE_POINTS.flatten()
    ee_tgt = EE_POINTS.flatten()

    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[:7],
        }
    }

    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)


if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentROSControlArm,
    'data_files_dir': EXP_DIR + 'data_files/',
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': 100,
    'target_end_effector': 'human_hand',
    # 'x0': x0s,
    # 'x0': np.concatenate([np.array([-0.06858,0.70147,0.08587,-2.119857,-0.049951,2.9128,0.7915173]),
    #                       np.zeros(7)]),
    'x0': np.concatenate([np.array([0.1, 0.8, 0.1, -2, -0.1, 3, 0.9]),
                          np.zeros(7)]),                     
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
}

algorithm = {
    'type': AlgorithmBADMM,
    'conditions': common['conditions'],
    'iterations': 11,
    'lg_step_schedule': np.array([1e-4, 1e-3, 1e-2, 1e-2]),
    'policy_dual_rate': 0.1,
    'init_pol_wt': 0.002,
    'ent_reg_schedule': np.array([1e-3, 1e-3, 1e-2, 1e-1]),
    'fixed_lg_step': 3,
    'kl_step': 1.0,
    'min_step_mult': 0.01,
    'max_step_mult': 3.0,
    'sample_decrease_var': 0.05,
    'sample_increase_var': 0.1,
    'policy_sample_mode': 'add'
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / FRANKA_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    # 'init_var': 1.0,
    'init_var': 0.5, #0.25,#1.0, # Kinda
    'stiffness': 1.0, #3,
    'stiffness_vel': 0.5, #0.25,
    # 'final_weight': 50,
    'dQ': SENSOR_DIMS[ACTION],
    'dt': agent['dt'],
    'T': agent['T'],
}

# algorithm['init_traj_distr'] = {
#     'type': init_pd,
#     'init_var': 0.1, #0.25,#1.0, # Kinda
#     'pos_gains':0.1,
#     'dQ': SENSOR_DIMS[ACTION],
#     'dt': agent['dt'],
#     'T': agent['T'],
# }

torque_cost = {
    'type': CostAction,
    'wu': 5e-3 / FRANKA_GAINS,
}


fk_cost = {
    'type': CostFK,
    'target_end_effector': 'human_hand',#np.array([0.0, 0.3, -0.5, 0.0, 0.3, -0.2]),
    'wp': np.array([1, 1, 1]),
    'l1': 2.0,
    'l2': 2.0,
    'alpha': 1e-5,
}

fk_cost1 = {
    'type': CostFK,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    #'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'target_end_effector':'human_hand',
    #'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'wp': np.array([1, 1, 1]),
    'l1': 0.1,
    'l2': 0.0001,
    # 'l1': 0, # Ville
    # 'l2': 1, # Ville
    'ramp_option': RAMP_LINEAR,
}

fk_cost2 = {
    'type': CostFK,
    #'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'target_end_effector':'human_hand',
    #'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'wp': np.array([1, 1, 1]),
    'l1': 1.0,
    'l2': 0.0,
    # 'l1': 0, # Ville
    # 'l2': 1.0, # Ville
    'wp_final_multiplier': 10.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    #'costs': [fk_cost1, fk_cost2],
    #'weights': [1.0, 1.0],
    'costs': [fk_cost],
    'weights': [1.0],
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

algorithm['policy_opt'] = {
    'type': PolicyOptCaffe,
    'weights_file_prefix': EXP_DIR + 'policy',
    'iterations': 3000,

}

algorithm['policy_prior'] = {
    'type': PolicyPriorGMM,
    'max_clusters': 20,
    'min_samples_per_cluster': 40,
    'max_samples': 40,
}

config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 1,
    'verbose_policy_trials': 1,
    'agent': agent,
    'gui_on': False,
    'algorithm': algorithm,
    'num_samples': 5,
}

common['info'] = generate_experiment_info(config)
