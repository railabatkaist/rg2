import unittest
import requests
import tempfile
import os
import shutil

from rg2 import Rg2Env
from rg2.utils import RgConfig

URDF_SOURCE_URL = "https://raw.githubusercontent.com/raisimTech/raisimLib/master/rsc/anymal/urdf/anymal.urdf"

CFG_YAML_STR = """
render: True
num_envs: 1
eval_every_n: 10
num_threads: 1
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
    def test_vecenv_sb3(self):

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

        # Test
        env = Rg2Env(tmp_urdf_path, cfgstr=cfg.as_yaml_str())

        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env

        check_env(env)

        assert env.num_envs == 1

        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=1000)

        env.close()

    def test_vecenv_sb3(self):

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
        cfgstr = RgConfig.from_yaml(tmp_yaml_path).as_yaml_str()
        env = Rg2Env(tmp_urdf_path, cfgstr=cfgstr)

        from stable_baselines3 import PPO

        assert env.num_envs == 1
        from stable_baselines3.common.env_util import make_vec_env

        envs = make_vec_env(
            Rg2Env, n_envs=8, env_kwargs={"urdf_path": tmp_urdf_path, "cfgstr": cfgstr}
        )

        model = PPO("MlpPolicy", envs, verbose=1)
        model.learn(total_timesteps=10_0000)
