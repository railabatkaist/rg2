import unittest
import requests
import tempfile
import os
import shutil

from rg2 import CVecEnv
from rg2 import VecEnv as VecEnv, SBVecGym
from rg2.utils import RgConfig

URDF_SOURCE_URL = "https://raw.githubusercontent.com/raisimTech/raisimLib/master/rsc/anymal/urdf/anymal.urdf"

CFG_YAML_STR = """
render: True
num_envs: 4
eval_every_n: 10
num_threads: 4
simulation_dt: 0.0025
control_dt: 0.01
max_time: 4.0
action_std: 0.3
reward:
    forwardVel:
        coeff: 0.3
    torque:
        coeff: -4e-5
"""


class VecEnvTester(unittest.TestCase):
    def test_vecenv_initialization(self):

        # Save URDF
        tmp_dir = tempfile.mkdtemp()
        tmp_urdf_path = os.path.join(tmp_dir, "anymal.urdf")
        with open(tmp_urdf_path, "w") as f:
            f.write(requests.get(URDF_SOURCE_URL).text)
        print("Saved URDF to: ", tmp_urdf_path)
        # SAVE YAML
        tmp_yaml_path = os.path.join(tmp_dir, "cfg.yaml")
        with open(tmp_yaml_path, "w") as f:
            f.write(CFG_YAML_STR)
        cfg = RgConfig.from_yaml(tmp_yaml_path)

        # test

        # from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvStepReturn, VecEnvWrapper
        # class SBVecEnv(VecEnv, RgVecEnv):

        #     def __init__(self, venv, ):
        #         super().__init__(venv = venv, observation_space=)

        env = SBVecGym(CVecEnv(tmp_urdf_path, cfg=cfg.as_yaml_str()))

        from stable_baselines3.common.env_checker import check_env

        # It will check your custom environment and output additional warnings if needed
        # If this causes error, this means you need to fix the error to work with SD3.
        # Check: https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html
        # check_env(env)

        from stable_baselines3 import A2C, PPO

        print(env.num_envs)

        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=10000000)
