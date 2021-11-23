""" This file defines an agent for the MuJoCo simulator environment. """
import copy

import numpy as np
import time
import mjcpy

from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_MUJOCO
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, \
        END_EFFECTOR_POINT_JACOBIANS, ACTION, RGB_IMAGE, RGB_IMAGE_SIZE, \
        CONTEXT_IMAGE, CONTEXT_IMAGE_SIZE, IMAGE_FEAT, \
        END_EFFECTOR_POINTS_NO_TARGET, END_EFFECTOR_POINT_VELOCITIES_NO_TARGET, NOISE

from gps.sample.sample import Sample


class AgentMuJoCo(Agent):
    """
    All communication between the algorithms and MuJoCo is done through
    this class.
    """
    def __init__(self, hyperparams):
        config = copy.deepcopy(AGENT_MUJOCO)
        config.update(hyperparams)
        Agent.__init__(self, config)
        self._setup_conditions()
        self._setup_world(hyperparams['filename'])

    def _setup_conditions(self):
        """
        Helper method for setting some hyperparameters that may vary by
        condition.
        """
        conds = self._hyperparams['conditions']
        # for field in ('x0','x0_mujoco', 'x0var', 'pos_body_idx', 'pos_body_offset', 'quat_body_offset', 'noisy_body_idx', 'noisy_body_var', 'filename'):
        #     self._hyperparams[field] = setup(self._hyperparams[field], conds)
        if self._hyperparams['test']:
            for field in ('x0','x0_mujoco', 'x0var', 'pos_body_idx', 'noisy_body_idx', 'noisy_body_var', 'filename', 'pos_human_test'):
                self._hyperparams[field] = setup(self._hyperparams[field], conds)
        else:
            for field in ('x0','x0_mujoco', 'x0var', 'pos_body_idx', 'noisy_body_idx', 'noisy_body_var', 'filename', 'pos_human'):
                self._hyperparams[field] = setup(self._hyperparams[field], conds)

    def _setup_world(self, filename):
        """
        Helper method for handling setup of the MuJoCo world.
        Args:
            filename: Path to XML file containing the world information.
        """
        self._world = []
        self._model = []
        self.nMotors = self._hyperparams['sensor_dims'][ACTION]
        self.shoulder_end = []
        self.elbow_end = []
        self.t_end = []

        # Initialize Mujoco worlds. If there's only one xml file, create a single world object,
        # otherwise create a different world for each condition.
        if not isinstance(filename, list):
            world = mjcpy.MJCWorld(filename)
            self._world = [world for _ in range(self._hyperparams['conditions'])]
            self._model = [self._world[i].get_model().copy()
                           for i in range(self._hyperparams['conditions'])]
        else:
            for i in range(self._hyperparams['conditions']):
                self._world.append(mjcpy.MJCWorld(self._hyperparams['filename'][i]))
                self._model.append(self._world[i].get_model())

        if self._hyperparams['human_ik']:
            for i in range(self._hyperparams['conditions']):
                # print(self._hyperparams['pos_human'][0][1])
                if self._hyperparams['test']:
                    [theta0, theta1, theta2] = self.inverse_kinematics_3dof(1.2, 0.3, 0.3, 1, self._hyperparams['pos_human_test'][i][0], self._hyperparams['pos_human_test'][i][1], self._hyperparams['pos_human_test'][i][2])
                else:
                    [theta0, theta1, theta2] = self.inverse_kinematics_3dof(1.2, 0.3, 0.3, 1, self._hyperparams['pos_human'][i][0], self._hyperparams['pos_human'][i][1], self._hyperparams['pos_human'][i][2])
                print("Human joint angles", [theta0, theta1, theta2])
                for j in range(len(self._hyperparams['pos_body_idx'][i])):
                    idx = self._hyperparams['pos_body_idx'][i][j]
                    self._model[i]['body_pos'][idx, :] = [0.8*np.cos(theta0), 0.8*np.sin(theta0), 0.0]
                    if 'quat_body_offset' in self._hyperparams:
                        self._model[i]['body_quat'][idx, :] = [np.cos((np.pi+theta0)/2),0,0,np.sin((np.pi+theta0)/2)]
                self.shoulder_end.append(theta1)
                self.elbow_end.append(theta2)
                self.t_end.append(self._hyperparams['t_end'][i])
                if not self._hyperparams['simulate_human']:
                    self._hyperparams['x0_mujoco'][i][1] = theta1
                    self._hyperparams['x0_mujoco'][i][2] = theta2
                # mj_X = self._hyperparams['x0_mujoco'][i]
        else:
            for i in range(self._hyperparams['conditions']):
                for j in range(len(self._hyperparams['pos_body_idx'][i])):
                    idx = self._hyperparams['pos_body_idx'][i][j]
                    self._model[i]['body_pos'][idx, :] += \
                            self._hyperparams['pos_body_offset'][i][j]
                    if 'quat_body_offset' in self._hyperparams:
                        self._model[i]['body_quat'][idx, :] += \
                                self._hyperparams['quat_body_offset'][i][j]
                self.shoulder_end.append(0.8*(0.5+np.random.random()))
                self.elbow_end.append(0.8*(0.5+np.random.random()))
                self.t_end.append((0.5+np.random.random())*100)


            mj_X = self._hyperparams['x0_mujoco'][i]
            if self._hyperparams['pickup']:
                if self._hyperparams['torque_control']:
                    mj_X = self.pick_object_torque_control(i, mj_X, False)
                else:
                    mj_X = self.pick_object_pos_control(i, mj_X, False)
            # self._hyperparams['x0'] = setup(self._hyperparams['x0'], i+1)
        # print("X0", self._hyperparams['x0'])
        if self._hyperparams['reduced'] == "FullState":
            self._joint_idx = list(range(self._model[0]['nq']))
            self._vel_idx = [i + self._model[0]['nq'] for i in range(self._model[0]['nv'])]
        elif self._hyperparams['reduced'] == "Only7Joints" or self._hyperparams['reduced'] == "Only7JointsRelativeEEF" :
            self._joint_idx = list(range(4,11))
            self._vel_idx = [i + self._model[0]['nq'] for i in range(4,11)]
        else:
            self._joint_idx = list(range(4,13))
            self._vel_idx = [i + self._model[0]['nq'] for i in range(4,13)]

        # print("Joint idx=",self._joint_idx)

        # Initialize x0.
        self.x0 = []
        for i in range(self._hyperparams['conditions']):
            if END_EFFECTOR_POINTS in self.x_data_types:
                # TODO: this assumes END_EFFECTOR_VELOCITIES is also in datapoints right?
                self._init(i)
                if self._hyperparams['reduced'] == "RelativeEEF" or self._hyperparams['reduced'] == "Only7JointsRelativeEEF":
                    if self._hyperparams['reduced'] == "RelativeEEF":
                        sites = 9
                    else:
                        sites = 3
                    eepts = self._world[i].get_data()['site_xpos'].flatten()
                    eepts_rel = eepts[0:2*sites]
                    for site in range(0,sites-sites/3):
                        eepts_rel[3*site:3*site+3] = eepts[3*site:3*site+3] - eepts[2*sites:2*sites+3]
                    # print("End effector points:", eepts)
                    if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                        self.x0.append(
                            np.concatenate([self._hyperparams['x0'][i], np.tile(eepts_rel,self._hyperparams['pre_timesteps']), np.zeros_like(np.tile(eepts_rel,self._hyperparams['pre_timesteps']))])
                        )
                    else:
                        self.x0.append(
                            np.concatenate([self._hyperparams['x0'][i], np.tile(eepts_rel,self._hyperparams['pre_timesteps'])])
                        )
                else:
                    eepts = self._world[i].get_data()['site_xpos'].flatten()
                    if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                        self.x0.append(
                            np.concatenate([self._hyperparams['x0'][i], np.tile(eepts, self._hyperparams['pre_timesteps']), np.zeros_like(np.tile(eepts,self._hyperparams['pre_timesteps']))])
                        )
                        print("Reached here, reached here", self.x0)
                        # print("EEPTS=", np.tile(eepts,self._hyperparams['pre_timesteps']) )
                    else:
                        self.x0.append(
                            np.concatenate([self._hyperparams['x0'][i], np.tile(eepts,self._hyperparams['pre_timesteps'])]))
            elif END_EFFECTOR_POINTS_NO_TARGET in self.x_data_types:
                self._init(i)
                eepts = self._world[i].get_data()['site_xpos'].flatten()
                eepts_notgt = np.delete(eepts, self._hyperparams['target_idx'])
                self.x0.append(
                    np.concatenate([self._hyperparams['x0'][i], eepts_notgt, np.zeros_like(eepts_notgt)])
                )
            else:
                self.x0.append(self._hyperparams['x0'][i])
            if IMAGE_FEAT in self.x_data_types:
                self.x0[i] = np.concatenate([self.x0[i], np.zeros((self._hyperparams['sensor_dims'][IMAGE_FEAT],))])

        cam_pos = self._hyperparams['camera_pos']
        for i in range(self._hyperparams['conditions']):
            self._world[i].init_viewer(AGENT_MUJOCO['image_width'],
                                       AGENT_MUJOCO['image_height'],
                                       cam_pos[0], cam_pos[1], cam_pos[2],
                                       cam_pos[3], cam_pos[4], cam_pos[5])

    def sample(self, policy, condition, iteration, sample_number, verbose=True, save=True, noisy=True):
        """
        Runs a trial and constructs a new sample containing information
        about the trial.
        Args:
            policy: Policy to to used in the trial.
            condition: Which condition setup to run.
            verbose: Whether or not to plot the trial.
            save: Whether or not to store the trial into the samples.
            noisy: Whether or not to use noise during sampling.
        """
        ##Test case
        # print(iteration)
        # if self._hyperparams['test'] and verbose:
        #     idx = self._hyperparams['pos_body_idx']
        #     self._model[condition]['body_pos'][idx, :] += -self._hyperparams['pos_body_offset'][condition][0]
        #     if 'quat_body_offset' in self._hyperparams:
        #         self._model[condition]['body_quat'][idx, :] += -self._hyperparams['quat_body_offset'][condition][0]

        #     self._model[condition]['body_pos'][idx, :] += self._hyperparams['pos_body_test_offset'][condition]
        #     self._model[condition]['body_quat'][idx, :] += self._hyperparams['quat_body_test_offset'][condition]

        # Create new sample, populate first time step.
        feature_fn = None
        if 'get_features' in dir(policy):
            feature_fn = policy.get_features
        new_sample = self._init_sample(condition, feature_fn=feature_fn)
        mj_X = self._hyperparams['x0_mujoco'][condition]
        # print("State", mj_X)
        U = np.zeros([self.T, self._model[condition]['nu']])
        #U = np.zeros([self.T, self.dU])
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))
        if np.any(self._hyperparams['x0var'][condition] > 0):
            x0n = self._hyperparams['x0var'] * \
                    np.random.randn(self._hyperparams['x0var'].shape)
            mj_X += x0n
        noisy_body_idx = self._hyperparams['noisy_body_idx'][condition]
        if noisy_body_idx.size > 0:
            for i in range(len(noisy_body_idx)):
                idx = noisy_body_idx[i]
                var = self._hyperparams['noisy_body_var'][condition][i]
                self._model[condition]['body_pos'][idx, :] += \
                        var * np.random.randn(1, 3)

        # if self._hyperparams['pickup']:
        #     if self._hyperparams['jaco']:
        #         mj_X = self.pick_object_jaco(condition, mj_X, verbose)
        #     elif self._hyperparams['torque_control']:
        #         mj_X = self.pick_object_torque_control(condition, mj_X, verbose)
        #     else:
        #         mj_X = self.pick_object_pos_control(condition, mj_X, verbose)
        np.random.seed()
        shoulder_start = 0
        elbow_start = 0
        if self._hyperparams['random_simulate_human']:
            shoulder_end = 0.8*(0.5+np.random.random())
            elbow_end = 0.8*(0.5+np.random.random())
            t_end = (0.5+np.random.random())*100
        else:
            shoulder_end = self.shoulder_end[condition]
            elbow_end = self.elbow_end[condition]
            t_end = self.t_end[condition]
        # print("Shoulder end =", shoulder_end)
        # print("Elbow end=", elbow_end)
        # print("Time end=", t_end)
        # Take the sample.
        distance_data_file = open(self._hyperparams['data_files_dir']+'distance_data.txt', 'a')
        gripper_data_file = open(self._hyperparams['data_files_dir']+'gripper_data.txt', 'a')

        for t in range(self.T):
            X_t = new_sample.get_X(t=t)
            obs_t = new_sample.get_obs(t=t)
            #mj_U = policy.act(X_t, obs_t, t, noise[t, :])
            mj_U = np.concatenate((policy.act(X_t, obs_t, t, noise[t, :]),np.zeros(self._model[condition]['nu'] - self.nMotors)))
            for joint in range(self.nMotors):
                mj_U[joint] = max(-3.0, min(mj_U[joint], 3.0))
            # print("Mujoco state", X_t)
            print("Mujoco control input", mj_U)
            if self._hyperparams['simulate_human']:
                #qpos = self._world[condition].get_data()["qpos"]
                #qpos[5] = 0.5
                #self._world[condition].set_data({"qpos": qpos})
                #print(self._world[condition].get_data()["qpos"])
                if t < t_end:
                    shoulder = 0.000905*(shoulder_end-shoulder_start)/(0.0008908+np.exp(-12.87*t/t_end))
                    elbow = 0.000905*(elbow_end-elbow_start)/(0.0008908+np.exp(-12.87*t/t_end))

                    #elbow = elbow_end + (elbow_start-elbow_end)*(23.2*pow(t/t_end,7) + 34.2*pow(t/t_end,6) -240.9*pow(t/t_end,5) + 314.6*pow(t/t_end,4) -157.3*pow(t/t_end,3) + 27.2 *pow(t/t_end,2) - 1.7*pow(t/t_end,1) + 1.0)
                    #elbow = elbow_end + (elbow_start-elbow_end)*(115.1*(t/t_end)**7 -376.9*(t/t_end)**6+454.1*(t/t_end)**5 -240.1*(t/t_end)**4+53.1*(t/t_end)**3 -6.7 * (t/t_end)**2 + 0.5*(t/t_end) + 1.0)
                    mj_X[1] = shoulder
                    mj_X[2] = elbow
                else:
                    #shoulder = 0.000905*(shoulder_end-shoulder_start)/(0.0008908+np.exp(-12.87))
                    #elbow = 0.000905*(elbow_end-elbow_end)/(0.0008908+np.exp(-12.87))
                    #elbow = elbow_end + (elbow_start-elbow_end)*(115.1 -376.9+454.1 -240.1+53.1 -6.7 + 0.5 + 1.1)
                    mj_X[1] = shoulder_end
                    mj_X[2] = elbow_end

                    #mj_X[1] = 0+(0.8)*t/self.T
                    #mj_X[2] = 0+(0.8)*t/self.T
            else:
                shoulder = shoulder_end
                elbow = elbow_end
                mj_X[1] = shoulder
                mj_X[2] = elbow
            # Compute distance between eef and hand
            # if self._hyperparams['simulate_human']:
            pt = self._world[condition].get_data()['site_xpos'].flatten()
            # print(pt)
            #import pdb; pdb.set_trace()
            # dist = pt[18:27] - pt[0:9]
            dist = pt[6:9] - pt[0:3]
            error = np.linalg.norm(dist)/3
            #Save distance between eef and hand to a file

            distance_data_file.write("%s," % error)
            #Save gripper position to a file
            grip_width = mj_X[11] + mj_X[12]
            gripper_data_file.write("%s," % grip_width)

            # if self._hyperparams['threshold']:
            #     if error < 0.1:
            #         mj_U[-2:] = [-2,-2]
            #     else:
            #         mj_U[-2:] = [2,2]
            U[t, :] = mj_U
            if verbose:
                self._world[condition].plot(mj_X)
            if (t + 1) < self.T:
                for _ in range(self._hyperparams['substeps']):
                    mj_X, _ = self._world[condition].step(mj_X, mj_U)
                self._data = self._world[condition].get_data()
                self._set_sample(new_sample, mj_X, t, condition, feature_fn=feature_fn)

        new_sample.set(ACTION, U[:,0:self.nMotors])
        #new_sample.set(ACTION, U)
        new_sample.set(NOISE, noise)
        if save:
            self._samples[condition].append(new_sample)
        distance_data_file.write('\n')
        distance_data_file.close()
        gripper_data_file.write('\n')
        gripper_data_file.close()
        return new_sample

    def _init(self, condition):
        """
        Set the world to a given model, and run kinematics.
        Args:
            condition: Which condition to initialize.
        """

        # Initialize world/run kinematics
        self._world[condition].set_model(self._model[condition])
        x0 = self._hyperparams['x0_mujoco'][condition]
        idx = len(x0) // 2
        data = {'qpos': x0[:idx], 'qvel': x0[idx:]}
        self._world[condition].set_data(data)
        self._world[condition].kinematics()

    def _init_sample(self, condition, feature_fn=None):
        """
        Construct a new sample and fill in the first time step.
        Args:
            condition: Which condition to initialize.
            feature_fn: funciton to comptue image features from the observation.
        """
        sample = Sample(self)

        # Initialize world/run kinematics
        self._init(condition)

        # Initialize sample with stuff from _data
        data = self._world[condition].get_data()
        if self._hyperparams['reduced']=="NoHumanJoints":
            sample.set(JOINT_ANGLES, data['qpos'][4:].flatten(), t=0)
            sample.set(JOINT_VELOCITIES, data['qvel'][4:].flatten(), t=0)
            eepts = data['site_xpos'].flatten()
            sample.set(END_EFFECTOR_POINTS, np.tile(eepts, self._hyperparams['pre_timesteps']), t=0)
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, np.zeros_like(np.tile(eepts,self._hyperparams['pre_timesteps'])), t=0)
            jac = np.zeros([eepts.shape[0], self._model[condition]['nv']])
            for site in range(eepts.shape[0] // 3):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.tile(jac, self._hyperparams['pre_timesteps']), t=0)
        elif self._hyperparams['reduced'] == "Only7Joints":
            sample.set(JOINT_ANGLES, data['qpos'][4:11].flatten(), t=0)
            sample.set(JOINT_VELOCITIES, data['qvel'][4:11].flatten(), t=0)
            eepts = data['site_xpos'].flatten()
            sample.set(END_EFFECTOR_POINTS, np.tile(eepts, self._hyperparams['pre_timesteps']), t=0)
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, np.zeros_like(np.tile(eepts,self._hyperparams['pre_timesteps'])), t=0)
            jac = np.zeros([eepts.shape[0], self._model[condition]['nv']])
            for site in range(eepts.shape[0] // 3):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.tile(jac, self._hyperparams['pre_timesteps']), t=0)
        elif self._hyperparams['reduced'] == 'Only7JointsRelativeEEF':
            sample.set(JOINT_ANGLES, data['qpos'][4:11].flatten(), t=0)
            sample.set(JOINT_VELOCITIES, data['qvel'][4:11].flatten(), t=0)
            eepts = data['site_xpos'].flatten()
            eepts_rel = eepts[0:6]
            for site in range(0,2):
                eepts_rel[3*site:3*site+3] = eepts[3*site:3*site+3] - eepts[6:9]
            sample.set(END_EFFECTOR_POINTS, np.tile(eepts_rel, self._hyperparams['pre_timesteps']), t=0)
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, np.zeros_like(np.tile(eepts_rel,self._hyperparams['pre_timesteps'])), t=0)
            jac = np.zeros([eepts_rel.shape[0], self._model[condition]['nv']])
            for site in range(0,2):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)- self._world[condition].get_jac_site(2)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.tile(jac, self._hyperparams['pre_timesteps']), t=0)
        elif self._hyperparams['reduced'] == "RelativeEEF":
            sample.set(JOINT_ANGLES, data['qpos'][4:].flatten(), t=0)
            sample.set(JOINT_VELOCITIES, data['qvel'][4:].flatten(), t=0)
            eepts = data['site_xpos'].flatten()
            eepts_rel = eepts[0:18]
            for site in range(0,6):
                eepts_rel[3*site:3*site+3] = eepts[3*site:3*site+3] - eepts[18:21]
            sample.set(END_EFFECTOR_POINTS, np.tile(eepts_rel, self._hyperparams['pre_timesteps']), t=0)
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, np.zeros_like(np.tile(eepts_rel,self._hyperparams['pre_timesteps'])), t=0)
            jac = np.zeros([eepts_rel.shape[0], self._model[condition]['nv']])
            for site in range(0,6):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)- self._world[condition].get_jac_site(6)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.tile(jac, self._hyperparams['pre_timesteps']), t=0)
        else:
            sample.set(JOINT_ANGLES, data['qpos'].flatten(), t=0)
            sample.set(JOINT_VELOCITIES, data['qvel'].flatten(), t=0)
            eepts = data['site_xpos'].flatten()
            sample.set(END_EFFECTOR_POINTS, np.tile(eepts, self._hyperparams['pre_timesteps']), t=0)
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, np.zeros_like(np.tile(eepts, self._hyperparams['pre_timesteps'])), t=0)

            if (END_EFFECTOR_POINTS_NO_TARGET in self._hyperparams['obs_include']):
                sample.set(END_EFFECTOR_POINTS_NO_TARGET, np.delete(eepts, self._hyperparams['target_idx']), t=0)
                sample.set(END_EFFECTOR_POINT_VELOCITIES_NO_TARGET, np.delete(np.zeros_like(eepts), self._hyperparams['target_idx']), t=0)

            jac = np.zeros([eepts.shape[0], self._model[condition]['nv']])
            for site in range(eepts.shape[0] // 3):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.tile(jac, self._hyperparams['pre_timesteps']), t=0)

        # save initial image to meta data
        self._world[condition].plot(self._hyperparams['x0_mujoco'][condition])
        img = self._world[condition].get_image_scaled(self._hyperparams['image_width'],
                                                      self._hyperparams['image_height'])
        # mjcpy image shape is [height, width, channels],
        # dim-shuffle it for later conv-net processing,
        # and flatten for storage
        img_data = np.transpose(img["img"], (2, 1, 0)).flatten()
        # if initial image is an observation, replicate it for each time step
        if CONTEXT_IMAGE in self.obs_data_types:
            sample.set(CONTEXT_IMAGE, np.tile(img_data, (self.T, 1)), t=None)
        else:
            sample.set(CONTEXT_IMAGE, img_data, t=None)
        sample.set(CONTEXT_IMAGE_SIZE, np.array([self._hyperparams['image_channels'],
                                                self._hyperparams['image_width'],
                                                self._hyperparams['image_height']]), t=None)
        # only save subsequent images if image is part of observation
        if RGB_IMAGE in self.obs_data_types:
            sample.set(RGB_IMAGE, img_data, t=0)
            sample.set(RGB_IMAGE_SIZE, [self._hyperparams['image_channels'],
                                        self._hyperparams['image_width'],
                                        self._hyperparams['image_height']], t=None)
            if IMAGE_FEAT in self.obs_data_types:
                raise ValueError('Image features should not be in observation, just state')
            if feature_fn is not None:
                obs = sample.get_obs()  # Assumes that the rest of the sample has been populated
                sample.set(IMAGE_FEAT, feature_fn(obs), t=0)
            else:
                # TODO - need better solution than setting this to 0.
                sample.set(IMAGE_FEAT, np.zeros((self._hyperparams['sensor_dims'][IMAGE_FEAT],)), t=0)
        return sample

    def _set_sample(self, sample, mj_X, t, condition, feature_fn=None):
        """
        Set the data for a sample for one time step.
        Args:
            sample: Sample object to set data for.
            mj_X: Data to set for sample.
            t: Time step to set for sample.
            condition: Which condition to set.
            feature_fn: function to compute image features from the observation.
        """
        sample.set(JOINT_ANGLES, np.array(mj_X[self._joint_idx]), t=t+1)
        sample.set(JOINT_VELOCITIES, np.array(mj_X[self._vel_idx]), t=t+1)

        if self._hyperparams['reduced'] == "RelativeEEF" or self._hyperparams['reduced'] == "Only7JointsRelativeEEF":
            if self._hyperparams['reduced'] == "RelativeEEF":
                sites = 9
            else:
                sites = 3
            cur_eepts = self._data['site_xpos'].flatten()
            cur_eepts_rel = cur_eepts[0:sites*2]
            for site in range(0,sites-sites/3):
                cur_eepts_rel[3*site:3*site+3] = cur_eepts[3*site:3*site+3] - cur_eepts[2*sites:2*sites+3]
            sample.set(END_EFFECTOR_POINTS, cur_eepts_rel, t=t+1)
            prev_eepts = sample.get(END_EFFECTOR_POINTS, t=t)
            eept_vels = (cur_eepts_rel - prev_eepts) / self._hyperparams['dt']
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                sample.set(END_EFFECTOR_POINT_VELOCITIES, eept_vels, t=t+1)
            jac = np.zeros([cur_eepts_rel.shape[0], self._model[condition]['nv']])
            for site in range(0,sites-sites/3):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)- self._world[condition].get_jac_site(sites-sites/3)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, jac, t=t+1)
        else:
            cur_eepts = self._data['site_xpos'].flatten()
            eepts =[]
            eepts = np.concatenate((cur_eepts, sample.get(END_EFFECTOR_POINTS, t=t)))
            sample.set(END_EFFECTOR_POINTS, eepts[:-9], t=t+1)
            prev_eepts = sample.get(END_EFFECTOR_POINTS, t=t)
            eept_vels = (cur_eepts - prev_eepts[0:9]) / self._hyperparams['dt']
            if END_EFFECTOR_POINT_VELOCITIES in self.x_data_types:
                eept_vels = np.concatenate((eept_vels, sample.get(END_EFFECTOR_POINT_VELOCITIES, t=t)))
                sample.set(END_EFFECTOR_POINT_VELOCITIES, eept_vels[:-9], t=t+1)

            if (END_EFFECTOR_POINTS_NO_TARGET in self._hyperparams['obs_include']):
                sample.set(END_EFFECTOR_POINTS_NO_TARGET, np.delete(cur_eepts, self._hyperparams['target_idx']), t=t+1)
                sample.set(END_EFFECTOR_POINT_VELOCITIES_NO_TARGET, np.delete(eept_vels, self._hyperparams['target_idx']), t=t+1)

            jac = np.zeros([cur_eepts.shape[0], self._model[condition]['nv']])
            #print("JACOBIANS=", jac.shape)
            for site in range(cur_eepts.shape[0] // 3):
                idx = site * 3
                jac[idx:(idx+3), :] = self._world[condition].get_jac_site(site)
            previous_jacs = sample.get(END_EFFECTOR_POINT_JACOBIANS, t = t)
            # print("JACOBIANS=", previous_jacs.shape)
            sample.set(END_EFFECTOR_POINT_JACOBIANS, np.concatenate((jac,previous_jacs[:,:-self._model[condition]['nv']]), axis = 1), t=t+1)
        if RGB_IMAGE in self.obs_data_types:
            img = self._world[condition].get_image_scaled(self._hyperparams['image_width'],
                                                          self._hyperparams['image_height'])
            sample.set(RGB_IMAGE, np.transpose(img["img"], (2, 1, 0)).flatten(), t=t+1)
            if feature_fn is not None:
                obs = sample.get_obs()  # Assumes that the rest of the observation has been populated
                sample.set(IMAGE_FEAT, feature_fn(obs), t=t+1)
            else:
                # TODO - need better solution than setting this to 0.
                sample.set(IMAGE_FEAT, np.zeros((self._hyperparams['sensor_dims'][IMAGE_FEAT],)), t=t+1)

    def _get_image_from_obs(self, obs):
        imstart = 0
        imend = 0
        image_channels = self._hyperparams['image_channels']
        image_width = self._hyperparams['image_width']
        image_height = self._hyperparams['image_height']
        for sensor in self._hyperparams['obs_include']:
            # Assumes only one of RGB_IMAGE or CONTEXT_IMAGE is present
            if sensor == RGB_IMAGE or sensor == CONTEXT_IMAGE:
                imend = imstart + self._hyperparams['sensor_dims'][sensor]
                break
            else:
                imstart += self._hyperparams['sensor_dims'][sensor]
        img = obs[imstart:imend]
        img = img.reshape((image_width, image_height, image_channels))
        return img

    def pick_object_jaco(self, condition, mj_X, verbose):
        # Close gripper
        target = np.array([-0.4, 0, -1.2, -2, 0, 3.8, 2.4, 0.014, 0.014])
        #while np.linalg.norm(mj_X[0:self.nMotors] - target) > 0.1:
        for i in range(100):
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = np.array([0, -100, 20, 0, 0, 0, -2.0, -2.0, -2.0])
            #mj_U = 10*(mj_X[0:self.nMotors] - target)
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)
        # Lift off
        for i in range(200):
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = np.array([0, 10, -20, 0, 0, 0, -2.0, -2.0,-2.0])
            #mj_U = 10*(mj_X[0:self.nMotors] - target)
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)
        # print("Initial Position =",mj_X)
        return mj_X

    def pick_object_torque_control(self, condition, mj_X, verbose):
        # Close gripper
        target = np.array([-0.4, 0, -1.2, -2, 0, 3.8, 2.4, 0.014, 0.014])
        #while np.linalg.norm(mj_X[0:self.nMotors] - target) > 0.1:
        for i in range(600):
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = np.array([0, 0, -10, 0, 0, 0, 0, -2.0,-2.0])
            #mj_U = 10*(mj_X[0:self.nMotors] - target)
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)
        # Lift off
        for i in range(600):
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = np.array([0, 0, -10, 87, 0, 12, 0, -1.0,-1.0])
            #mj_U = 10*(mj_X[0:self.nMotors] - target)
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)
        # print("Initial Position =",mj_X)
        return mj_X

    def pick_object_pos_control(self, condition, mj_X, verbose):
        # Hover over
        target = np.array([-0.4, 0, -1.2, -2, 0, 2, 2.4, 0.04, 0.04])
        while np.linalg.norm(mj_X[0:self.nMotors] - target) > 0.1:
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = target
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)

        # Go down
        target = np.array([-0.4, 0, -1.2, -2.4, 0, 2.4, 2.4, 0.04, 0.04])
        while np.linalg.norm(mj_X[0:self.nMotors] - target) > 0.1:
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = target
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)

        # Close gripper
        target = np.array([-0.4, 0, -1.2, -2.3, 0, 2.3, 2.4, 0.013, 0.013])
        while np.linalg.norm(mj_X[7]-target[7]) > 0.01 and np.linalg.norm(mj_X[8]-target[7]) > 0.01:
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = target
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)

        # Lift off
        target = np.array([-0.4, 0, -1.2, -1.8, 0, 3.8, 2.4, 0.013, 0.013])
        while np.linalg.norm(mj_X[0:self.nMotors] - target) > 0.1:
            if verbose:
                self._world[condition].plot(mj_X)
            mj_U = target
            for _ in range(self._hyperparams['substeps']):
                mj_X, _ = self._world[condition].step(mj_X, mj_U)

        return mj_X

    def inverse_kinematics_2dof(self, l0, l1, l2, r, px, py, pz):
        # x = np.sqrt(px**2+py**2+pz**2)
        x = r - np.sqrt(px**2+py**2)
        y = pz-l0
        theta2 = np.arccos(np.clip((x**2+y**2-l1**2-l2**2)/(2*l1*l2), -1, 1))
        theta1 = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2), l1+l2*np.cos(theta2))
        # print(theta1+np.pi/2)
        # print(theta2)
        # m = m +1
        return theta1+np.pi/2, theta2

    def inverse_kinematics_3dof(self, l0, l1, l2, r, px, py, pz):
        theta0 = np.arctan2(py,px)
        x_dash = r - np.sqrt(px**2+py**2)
        [theta1, theta2] = self.inverse_kinematics_2dof(l0, l1, l2, r, px, py, pz)
        return theta0, theta1, theta2

        #Close gripper
