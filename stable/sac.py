import gym
from gym import register
from functools import partial
from matlab_gym.mat_gym_mmap import MatlabGymMmapWrapper
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3 import A2C, DDPG, DQN, PPO, SAC, TD3
from stable_baselines3.common.env_util import make_vec_env
import numpy as np
import time
from multiprocessing import Process
import torch

def run_sac(seed):
    torch.set_num_threads(1)
    algo = 'sac'
    env_script = "make_1dof_env"
    env_partial = partial(MatlabGymMmapWrapper, "/home/sgillen/work/n_link_arm/", env_script, np.float32)
    register("n_link_arm-v0", entry_point = env_partial)

    env = make_vec_env("n_link_arm-v0", n_envs = 1, monitor_dir = f"./sb3_data/{env_script}/{algo}/{seed}")
    env = VecNormalize(env)

    model = SAC('MlpPolicy',
                env,
                verbose=2,
                seed = int(seed),
                tensorboard_log=f"./sb3_data/tensorboard/{env_script}",
                device='cpu',
                policy_kwargs={"net_arch":[64,64]},
                buffer_size=300000,
                batch_size=256,
                learning_starts=10000,
                use_sde=True,
                train_freq=64,
                gradient_steps=64,
                learning_rate=1e-3
                )
    
    model.learn(2e6)
    model.save(f"./sb3_data/{env_script}/{algo}/{seed}/model.pkl")
    env.save(f"./sb3_data/{env_script}/{algo}/{seed}/vecnormalize.pkl")


if __name__ == "__main__":
    p_list = []
    start = time.time()
    for seed in np.random.randint(0,2**32-1,8):
        p = Process(target=run_sac, args=(seed,))
        p.start()
        p_list.append(p)

    for p in p_list:
        p.join()


    print(f"experiment complete, total time: {time.time() - start}")
