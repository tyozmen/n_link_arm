import os
from matlab_gym.mat_gym_mmap import MatlabGymMmapWrapper 
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.env_util import make_vec_env
from functools import partial
from gym import register
from seagul.zoo3_utils import ALGOS
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import savemat
import pickle

env_script = "make_1dof_env"
exp_path = f"/home/sgillen/work/n_link_arm/stable/sb3_data/{env_script}"

env_partial = partial(MatlabGymMmapWrapper, "/home/sgillen/work/n_link_arm/", env_script)

try:
    register("bball_1dof-v0", entry_point = env_partial)
except:
    pass

model_dict = {}
norm_dict = {}
df_dict = {}

for algo_dir in os.scandir(exp_path):
    algo_name = algo_dir.path.split("/")[-1]
    ALGO = ALGOS[algo_name]
    
    if not algo_name in model_dict:
        model_dict[algo_name] = []
    if not algo_name in df_dict:
        df_dict[algo_name] = []
    if not algo_name in norm_dict:
        norm_dict[algo_name] = []
    
    for seed_dir in os.scandir(algo_dir.path):   
        seed =  seed_dir.path.split("/")[-1]
        df_dict[algo_name].append(pd.read_csv(f"{seed_dir.path}/0.monitor.csv", skiprows=1))
        model_dict[algo_name].append(ALGO.load(f"{seed_dir.path}/model.pkl", mapped_devic='cpu'))
    
        norm = pickle.load(open(f"{seed_dir.path}/vecnormalize.pkl", 'rb'))
        norm_dict[algo_name].append(norm)
        
        #env = make_vec_env('bball_1dof-v0', n_envs = 1)
        #env = VecNormalize.load(f"{seed_dir.path}/vecnormalize.pkl", env)
        #env.training = False
        #env.norm_reward = False
        #env_dict[algo_name].append(env)



