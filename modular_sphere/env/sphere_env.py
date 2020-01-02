#!/usr/bin/env python
# coding: utf-8
import sys
# CV2_ROS = '/opt/ros/kinetic/lib/python2.7/dist-packages'
# if CV2_ROS in sys.path:
#     sys.path.remove(CV2_ROS)
#     sys.path.append(CV2_ROS)
import gym
import time
import numpy as np
import random
from gym import spaces
import pybullet
from modular_sphere import RobotSphere
from modular_sphere import SimulationManager

OBS_DIM = 6
ACT_DIM = 2


class RobotSphereEnv(gym.Env):
    """
    Gym environment for the robot sphere to learn to reach another sphere
    """

    def __init__(self, gui=False):
        """
        Constructor

        Parameters:
            gui - boolean, the simulation is in DIRECT mode if set to False
            (default value)
        """
        # Passed to True at the end of an episode
        self.episode_over = False
        self.gui = gui
        self.simulation_manager = SimulationManager()
        self._setupScene()

        obs_space = np.inf * np.ones([OBS_DIM])
        self.observation_space = spaces.Box(
            low=-obs_space,
            high=obs_space)

        self.action_space = spaces.Box(
            low=np.array([-1]*ACT_DIM),
            high=np.array([1]*ACT_DIM))

    def step(self, action):
        """

        Parameters
        ----------
        action : list of velocities to be applied on the robot's joints

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing your observation of
                the environment.
            reward (float) :
                amount of reward achieved by the previous action. The scale
                varies between environments, but the goal is always to increase
                your total reward.
            episode_over (bool) :
                whether it's time to reset the environment again. Most (but not
                all) tasks are divided up into well-defined episodes, and done
                being True indicates the episode has terminated. (For example,
                perhaps the pole tipped too far, or you lost your last life.)
            info (dict) :
                 diagnostic information useful for debugging. It can sometimes
                 be useful for learning (for example, it might contain the raw
                 probabilities behind the environment's last state change).
                 However, official evaluations of your agent are not allowed to
                 use this for learning.
        """
        try:
            action = list(action)
            assert len(action) == ACT_DIM

        except AssertionError:
            print("Incorrect action")
            return None, None, None, None
        np.clip(action, [-1]*ACT_DIM,
                [1]*ACT_DIM)
        action.append(0)
        self.robot_sphere.move(np.multiply(action, 100))
        obs, reward = self._getState()
        return obs, reward, self.episode_over, {}

    def reset(self):
        """
        Resets the environment for a new episode
        """
        self.episode_over = False
        self._resetScene()

        obs, _ = self._getState()
        return obs

    def render(self, mode='human', close=False):
        pass

    def _getState(self, convergence_criteria=0.12, divergence_criteria=0.6):
        """
        Gets the observation and computes the current reward. Will also
        determine if the episode is over or not, by filling the episode_over
        boolean. When the euclidian distance between the wrist link and the
        cube is inferior to the one defined by the convergence criteria, the
        episode is stopped
        """

        # Fill the observation
        robot_sphere_pos = np.array(
            self.robot_sphere.getPosition(),
            dtype=np.float32
        )
        robot_sphere_goal_pos = np.array(
            self.robot_sphere_goal.getPosition(),
            dtype=np.float32
        )
        obs = np.concatenate([robot_sphere_pos] + [robot_sphere_goal_pos])
        # Fill reward
        reward = 0
        if self.time_init == 0:
            self.time_init = time.time()
        distance = np.linalg.norm(robot_sphere_goal_pos - robot_sphere_pos)
        collision_detected = list(pybullet.getContactPoints(
            self.robot_sphere.getRobotModel(),
            self.robot_sphere_goal.getRobotModel(),
            physicsClientId=self.robot_sphere.getPhysicsClientId()))
        if len(collision_detected) != 0:
            reward += 20
            reward -= time.time() - self.time_init
            self.episode_over = True
        if distance > 15 or time.time() - self.time_init > 10:
            reward += -20
            reward -= time.time() - self.time_init
            self.episode_over = True
        if self.distance_init == 0:
            self.distance_init = distance
        reward += self.distance_init - distance
        self.distance_init = distance
        return obs, reward

    def _setupScene(self):
        """
        Setup a scene environment within the simulation
        """
        self.client = self.simulation_manager.launchSimulation(gui=self.gui)
        self.robot_sphere = self.simulation_manager.spawnRobotSphere(
            self.client,
            translation=[0,0,1], quaternion=[0,0,0,1],
            spawn_ground_plane=True)
        self.robot_sphere_goal = self.simulation_manager.spawnRobotSphere(
            self.client,
            translation=[
                [-1,1][random.randrange(2)] * random.randrange(5, 10),
                [-1,1][random.randrange(2)] * random.randrange(5, 10), 1
            ],
            quaternion=[0,0,0,1],
            spawn_ground_plane=True)
        self.robot_sphere_goal.setColor([255, 0, 0, 0.8])
        self.distance_init = 0
        self.time_init = 0

    def _resetScene(self):
        """
        Resets the scene for a new scenario
        """
        pybullet.resetBasePositionAndOrientation(
            self.robot_sphere.getRobotModel(),
            [0,0,1], [0,0,0,1],
            physicsClientId=self.robot_sphere.getPhysicsClientId()
        )
        pybullet.resetBasePositionAndOrientation(
            self.robot_sphere_goal.getRobotModel(),
            [
                [-1,1][random.randrange(2)] * random.randrange(5, 10),
                [-1,1][random.randrange(2)] * random.randrange(5, 10), 1
            ],
            [0,0,0,1],
            physicsClientId=self.robot_sphere.getPhysicsClientId()
        )
        self.distance_init = 0
        self.time_init = 0

    def _termination(self):
        """
        Terminates the environment
        """
        self.simulation_manager.stopSimulation(self.client)


