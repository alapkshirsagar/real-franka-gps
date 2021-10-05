""" This file defines the joint velocity cost. """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_JOINT_VELOCITY
from gps.algorithm.cost.cost import Cost
from gps.proto.gps_pb2 import JOINT_VELOCITIES

class CostJointVelocity(Cost):
    """ Computes joint velocity penalties. """
    def __init__(self, hyperparams):
        config = copy.deepcopy(COST_JOINT_VELOCITY)
        config.update(hyperparams)
        Cost.__init__(self, config)

    def eval(self, sample):
        """
        Evaluate cost function and derivatives on a sample.
        Args:
            sample: A single sample
        """
        sample_v = sample.get(JOINT_VELOCITIES)
        T = sample.T
        Du = sample.dU
        Dx = sample.dX
        l = 0.5 * np.sum(self._hyperparams['wv'] * (sample_v ** 2), axis=1)
        lu = np.zeros((T, Du))
        lx = np.zeros((T, Dx))
        lx[:,7:14] = self._hyperparams['wv']*sample_v
        luu = np.zeros((T, Du, Du))
        lxx = np.zeros((T, Dx, Dx))
        diagonal = np.zeros(Dx)
        diagonal[7:14] = self._hyperparams['wv']
        lxx = np.tile(np.diag(diagonal), [T, 1, 1])
        lux = np.zeros((T, Du, Dx))
        return l, lx, lu, lxx, luu, lux
