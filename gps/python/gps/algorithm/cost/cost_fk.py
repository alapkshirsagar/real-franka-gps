""" This file defines the forward kinematics cost function. """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_FK
from gps.algorithm.cost.cost import Cost
from gps.algorithm.cost.cost_utils import get_ramp_multiplier
from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS


class CostFK(Cost):
    """
    Forward kinematics cost function. Used for costs involving the end
    effector position.
    """
    def __init__(self, hyperparams):
        config = copy.deepcopy(COST_FK)
        config.update(hyperparams)
        Cost.__init__(self, config)

    def eval(self, sample):
        """
        Evaluate forward kinematics (end-effector penalties) cost.
        Temporary note: This implements the 'joint' penalty type from
            the matlab code, with the velocity/velocity diff/etc.
            penalties removed. (use CostState instead)
        Args:
            sample: A single sample.
        """
        T = sample.T
        dX = sample.dX
        dU = sample.dU

        wpm = get_ramp_multiplier(
            self._hyperparams['ramp_option'], T,
            wp_final_multiplier=self._hyperparams['wp_final_multiplier']
        )
        wp = self._hyperparams['wp'] * np.expand_dims(wpm, axis=-1)

        # Initialize terms.
        l = np.zeros(T)
        lu = np.zeros((T, dU))
        lx = np.zeros((T, dX))
        luu = np.zeros((T, dU, dU))
        lxx = np.zeros((T, dX, dX))
        lux = np.zeros((T, dU, dX))

        # Choose target.
        tgt = self._hyperparams['target_end_effector']
        # if tgt == 'human_hand':
        pt = sample.get(END_EFFECTOR_POINTS)
        #print("Points = ", pt)
        # print("Targets =", tgt)
        dist = pt[:,6:9] - pt[:,0:3]
        # print("Distance", dist)
        jx_full = sample.get(END_EFFECTOR_POINT_JACOBIANS) #for mujoco
        #jx_full = np.zeros((T, 3*9, dX)) #for the real robot
        jx = jx_full[:, 6:9, -sample.get(JOINT_ANGLES).shape[1]:]
        #print(jx_full.shape)
        #print("jx", jx.shape)
        # print("Points = ",pt[:, 6:9])
        # print("JOINT_ANGLES",sample.get(JOINT_ANGLES))
        # thetas = sample.get(JOINT_ANGLES)
        # for t in range(0,T-1):
        #     for joint in range(0,7):
        #         for coordinate in range(0,3):
        #             #jx[t,coordinate,joint] = ((pt[t,coordinate]-pt[t-1,coordinate])/(thetas[t,joint]-thetas[t-1, joint]))
        #             jx[t,coordinate,joint] = ((pt[t+1,coordinate]-pt[t,coordinate])/(thetas[t+1,joint]-thetas[t, joint]))
        #             # print("jx",jx[t,coordinate,joint])
        #             # print(pt[t+1,coordinate])
        #             # print(pt[t,coordinate])
        #             # print(thetas[t+1,joint])
        #             # print(thetas[t,joint])
        # # print("jx",jx )


        
        # TODO - These should be partially zeros so we're not double
        #        counting.
        #        (see pts_jacobian_only in matlab costinfos code)

        # Evaluate penalty term. Use estimated Jacobians and no higher
        # order terms.
        jxx_zeros = np.zeros((T, dist.shape[1], jx.shape[2], jx.shape[2]))
        l, ls, lss = self._hyperparams['evalnorm'](
            wp, dist, jx, jxx_zeros, self._hyperparams['l1'],
            self._hyperparams['l2'], self._hyperparams['alpha']
        )
        # Add to current terms.
        sample.agent.pack_data_x(lx, ls, data_types=[JOINT_ANGLES])
        sample.agent.pack_data_x(lxx, lss,
                                 data_types=[JOINT_ANGLES, JOINT_ANGLES])

        return l, lx, lu, lxx, luu, lux
