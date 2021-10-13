#Hyperparameters for handover task with Franka Robot BADMM
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.mjc.agent_mjc import AgentMuJoCo

from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction

from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_state import CostState
from gps.algorithm.cost.cost_utils import RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy_opt.policy_opt_caffe import PolicyOptCaffe
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM
from gps.algorithm.policy.policy_prior import PolicyPrior
# from gps.algorithm.policy_opt.policy_opt_tf import PolicyOptTf
# from gps.algorithm.policy_opt.tf_model_example import tf_network
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION

from gps.gui.config import generate_experiment_info
from gps.algorithm.cost.cost_utils import RAMP_CONSTANT, RAMP_QUADRATIC, evallogl2term, RAMP_LINEAR



ALGORITHM_NN_LIBRARY = "caffe"








SENSOR_DIMS = {
    JOINT_ANGLES: 9, #9 for robot, 4 for human hand
    JOINT_VELOCITIES: 9,
    END_EFFECTOR_POINTS: 18, #9x2 for robot eef, 9x2 for human hand, 9x2 for object
    END_EFFECTOR_POINT_VELOCITIES: 18,
    ACTION: 9,
}

Franka_Gains = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1])



BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
#EXP_DIR = BASE_DIR + '/../experiments/franka_torque_h2r_badmm/'
EXP_DIR = '/media/franka2/DATA/Kuka_Franka_Experiment_Data/franka_torque_h2r_badmm/'

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 8,
}

if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])



agent = {
    'type': AgentMuJoCo,
    'filename': './mjc_models/franka-torque-control-h2r-human-simulate-alap/franka_panda.xml',
    'data_files_dir': EXP_DIR + 'data_files/',
    #'filename': './mjc_models/pr2_arm3d.xml',
    'x0': np.concatenate([np.array([-2, 0, -1.2, -2, 0, 3.8, 1.8, 0.04, 0.04]),
                          np.zeros(9)]), #These values correspond to the joint angles and velocities
    'x0_mujoco': np.concatenate([np.array([1.2, 0 , 0, 0,-2, 0, -1.2, -2, 0, 3.8, 1.8, 0.04, 0.04]),
                          np.zeros(13)]), #These values correspond to the joint angles and velocities
    'pickup': False,
    'simulate_human': False,
    'random_simulate_human': False,
    'test': False,
    'human_ik': True,
    'pos_human': [np.array([-r*np.cos(theta*np.pi/180),-r*np.sin(theta*np.pi/180),z]) for r in [0.6]
                        for theta in np.linspace(0, 90, num = 2)
                            for z in np.linspace(0.9, 1.2, num = 4)],
    # 'pos_human_test': [np.array([0, -0.6, 1.2]), np.array([0, -0.9, 1.2]), np.array([0, -0.9, 0.9]), np.array([0, -0.6, 0.9]), np.array([-0.6, 0.0, 1.2]), np.array([-0.9, 0.0, 1.2]), np.array([-0.9, 0.0, 0.9]), np.array([-0.6, 0.0, 0.9]), np.array([-0.6, 0.0, 0.9]), np.array([-0.6, 0.0, 0.9])],
    'pos_human_test' : [np.array([-r*np.cos(theta*np.pi/180),-r*np.sin(theta*np.pi/180),z]) for r in np.linspace(0.6, 0.9, num = 2)
                        for theta in np.linspace(0, 90, num = 2)
                            for z in np.linspace(0.9, 1.2, num = 2)],
    'reduced': 'RelativeEEF',#FullState, NoHumanJoints, RelativeEEF
    'pre_timesteps': 1, #1+number of past timesteps to include
    'dt': 0.05,
    'substeps': 5,
    'conditions': common['conditions'],
    'pos_body_idx': np.array([2]),
    'pos_body_offset': [[np.array([-1, -1.3, 0])], [np.array([-1.919, -0.919, 0])], [np.array([-2.3, 0, 0])], [np.array([-1, -1.3, 0])], [np.array([-1.95, -0.95, 0])],  [np.array([-1.95,  0.95, 0])], [np.array([-1, 1.3, 0])], [np.array([0, 0.2, 0])]],
    'quat_body_offset': [[np.array([1, 0, 0, 0])], [np.array([1, 0, 0, 0])], [np.array([0.707, 0, 0, -0.707])], [np.array([0, 0, 0, -1.0])], [np.array([0.707, 0, 0, -0.707])],  [np.array([-0.707, 0, 0, -0.707])], [np.array([-1, 0, 0, 0])], [np.array([0, 0, 0, 1])]],
    'shoulder_end': [0.4, 0.4, 1.2, 1.2, 0.4, 0.4, 1.2, 1.2],
    'elbow_end':     [1.2, 1.2, 0.4, 1.2, 0.4, 1.2, 0.4, 1.2],
    't_end': 100*np.ones(common['conditions']),
    'pos_body_test_offset': [[np.array([-2.280, -0.226, 0])], [np.array([-2.295, -0.113, 0])], [np.array([-2.3, 0, 0])]],#, [np.array([-1.3, 1.1, 0])], [np.array([-2.1, 0.5, 0])], [np.array([-1.75,  1.25, 0])]],
    'quat_body_test_offset': [[np.array([0.174, 0, 0, -0.985])], [np.array([0.087, 0, 0, -0.996])], [np.array([0, 0, 0, -1.0])]],#, [np.array([-1, 0, 0, 0])], [np.array([0, 0, 0, -1])], [np.array([-0.707, 0, 0, -0.707])]],
    'T': 20,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES],
    'camera_pos': np.array([-4, 4, 2.5, 90, -90, -45]),
}

algorithm = {
    'type': AlgorithmBADMM,
    'conditions': common['conditions'],
    'iterations': 12,
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
    'policy_sample_mode': 'replace'
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / Franka_Gains,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1000,
    'stiffness': 1.0,
    'stiffness_vel': 0.5,
    'dQ': SENSOR_DIMS[ACTION],
    'dt': agent['dt'],
    'T': agent['T'],
}

torque_cost = {
    'type': CostAction,
    'wu': 5e-5 / Franka_Gains,
}

fk_cost = {
    'type': CostFK,
    'target_end_effector': 'human_hand',#np.array([0.0, 0.3, -0.5, 0.0, 0.3, -0.2]),
    'wp': np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]),
    'l1': 2.0,
    'l2': 2.0,
    'alpha': 1e-5,
}

state_cost_open = {
    'type': CostState,
    # 'ramp_option': RAMP_INV_QUADRATIC,
    'data_types' : {
        JOINT_ANGLES: {
            'wp': np.array([0, 0, 0, 0, 0, 0, 0, 1, 1]),
            'target_state': np.array([-0.4, 0, -1.2, -2.4, 0, 2.4, 2.4, 0.04, 0.04]),
        },
    },
}

state_cost_close = {
    'type': CostState,
    # 'ramp_option': RAMP_QUADRATIC,
    'data_types' : {
        JOINT_ANGLES: {
            'wp': np.array([0, 0, 0, 0, 0, 0, 0, 1, 1]),
            'target_state': np.array([-0.4, 0, -1.2, -2.4, 0, 2.4, 2.4, 0.015, 0.015]),
        },
    },
}


algorithm['cost'] = {
    'type': CostSum,
    'costs': [fk_cost],#[torque_cost, handover_cost, fk_cost],
    'weights': [1.0],#[0.0, 0.0, 1.0],
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

if ALGORITHM_NN_LIBRARY == "tf":
    algorithm['policy_opt'] = {
        'type': PolicyOptTf,
        'network_params': {
            'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
            'sensor_dims': SENSOR_DIMS,
        },
        'weights_file_prefix': EXP_DIR + 'policy',
        'iterations': 3000,
        'network_model': tf_network
    }
elif ALGORITHM_NN_LIBRARY == "caffe":
    algorithm['policy_opt'] = {
        'type': PolicyOptCaffe,
        'weights_file_prefix': EXP_DIR + 'policy',
        'iterations': 5000,
    }

algorithm['policy_prior'] = {
    'type': PolicyPriorGMM,
    'max_clusters': 20,
    'min_samples_per_cluster': 40,
    'max_samples': 20,
}

config = {
    'iterations': algorithm['iterations'],
    'num_samples': 5,
    'verbose_trials': 1,
    'verbose_policy_trials': 1,
    'common': common,
    'agent': agent,
    'gui_on': False,
    'algorithm': algorithm,
}

common['info'] = generate_experiment_info(config)
