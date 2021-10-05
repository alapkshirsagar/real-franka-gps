""" This file defines the torque (action) cost. """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_ACTION
from gps.algorithm.cost.cost import Cost


class CostAction(Cost):
    """ Computes torque penalties. """
    def __init__(self, hyperparams):
        config = copy.deepcopy(COST_ACTION)
        config.update(hyperparams)
        Cost.__init__(self, config)

    def eval(self, sample):
        """
        Evaluate cost function and derivatives on a sample.
        Args:
            sample: A single sample
        """
        # sample_u = sample.get_U()
        # T = sample.T
        # Du = sample.dU
        # Dx = sample.dX
        # l = 0.5 * np.sum(self._hyperparams['wu'] * (sample_u ** 2), axis=1)
        # lu = self._hyperparams['wu'] * sample_u
        # lx = np.zeros((T, Dx))
        # luu = np.tile(np.diag(self._hyperparams['wu']), [T, 1, 1])
        # lxx = np.zeros((T, Dx, Dx))
        # lux = np.zeros((T, Du, Dx))


        sample_u = sample.get_U()
        T = sample.T
        Du = sample.dU
        Dx = sample.dX
        l = np.zeros(T)
        torque_cost =  0.5 * np.sum(self._hyperparams['wu'] * (sample_u ** 2), axis=1)
        l[0] = torque_cost[0]
        lu = np.zeros((T,Du)) 
        lu[0,:] = self._hyperparams['wu'] * sample_u[0,:]
        lx = np.zeros((T, Dx))
        luu = np.zeros((T, Du, Du))
        luu[0,:,:] = np.tile(np.diag(self._hyperparams['wu']), [1, 1, 1])
        lxx = np.zeros((T, Dx, Dx))
        lux = np.zeros((T, Du, Dx))
        return l, lx, lu, lxx, luu, lux
