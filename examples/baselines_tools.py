#!/usr/bin/env python3
from datetime import datetime
from env.sphere_env import RobotSphereEnv
import numpy as np
import time

from stable_baselines.common.policies import MlpPolicy as MlpLstmPolicy
from stable_baselines.ddpg.policies import MlpPolicy as DDPGMlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from stable_baselines.ddpg.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines import DDPG

PATH_MODEL_1 = 'examples/data/models/'
PATH_MODEL_2 = "data/models/"
AGENT = "PPO2"
NB_ITER = int(2e6)

def init_model(gui=True):
    env = RobotSphereEnv(gui=gui)
    env = DummyVecEnv([lambda: env])
    if AGENT is "PPO2":
        model = PPO2(
                 MlpLstmPolicy,
                 env,
                 n_steps=4096,
                 verbose=2,
                 tensorboard_log="logs/" + AGENT + "Agent/" +
                 datetime.now().strftime(
                     "%Y%m%d-%H%M%S"))
    if AGENT is "DDPG":
        action_noise = OrnsteinUhlenbeckActionNoise(
                    mean=np.zeros(env.action_space.shape[-1]),
                    sigma=float(0.5) * np.ones(env.action_space.shape[-1]))

        model = DDPG(
            DDPGMlpPolicy,
            env,
            verbose=2,
            param_noise=None,
            action_noise=action_noise,
            tensorboard_log="logs/" + AGENT + "Agent/" +
            datetime.now().strftime(
                "%Y%m%d-%H%M%S"))
    return env, model


def train(num_timesteps, seed, model_path=None):

    env, model = init_model()

    i = 0
    save_each_nb_iter = int(1e6)
    if NB_ITER < save_each_nb_iter:
        save_each_nb_iter = NB_ITER
    while i < num_timesteps:
        if i != 0:
            model.load(model_path + "/" + AGENT + "_" + repr(i))
        model.learn(total_timesteps=save_each_nb_iter)
        i += save_each_nb_iter
        model.save(model_path + "/" + AGENT + "_" + repr(i))
    env.close()

def visualize(name_model):
    model = getattr(globals()[AGENT], 'load')(name_model)
    env = RobotSphereEnv(gui=True)
    env = DummyVecEnv([lambda: env])
    # Enjoy trained agent
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, _, _, _ = env.step(action)
        env.render()
