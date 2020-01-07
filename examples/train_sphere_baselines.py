#!/usr/bin/env python3

from datetime import datetime
from env.sphere_env import RobotSphereEnv
import numpy as np
import time

from stable_baselines.common.policies import MlpPolicy as PPO2MlpPolicy
from stable_baselines.common.policies import MlpPolicy as MlpLstmPolicy
from stable_baselines.ddpg.policies import MlpPolicy as DDPGMlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from stable_baselines.ddpg.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines import DDPG
from stable_baselines.gail import generate_expert_traj
from stable_baselines.gail import ExpertDataset

PATH_MODEL = 'data/models/'
AGENT = "PPO2"


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
    while i < num_timesteps:
        if i != 0:
            model.load(model_path + "/" + AGENT + "_" + repr(i))
        model.learn(total_timesteps=int(1e6))
        i += int(1e6)
        model.save(model_path + "/" + AGENT + "_" + repr(i))
    env.close()

def visualize(name_model):
    if AGENT is "PPO2":
        model = PPO2.load(name_model)
    if AGENT is "DDPG":
        model = DDPG.load(name_model)
    env = RobotSphereEnv(gui=True)
    env = DummyVecEnv([lambda: env])
    # Enjoy trained agent
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, _, _, _ = env.step(action)
        env.render()


def main():
    seed = int(time.time())
    np.random.seed(seed)
    # train the model
    nb_iter = int(2e6)
    train(num_timesteps=nb_iter, seed=seed,
          model_path=PATH_MODEL)
    # visualize(PATH_MODEL + "/" + AGENT + "_" + str(nb_iter) + ".pkl")

if __name__ == '__main__':
    main()
