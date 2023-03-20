import numpy as np
import platform
import os

import gym
from gym import spaces
from ..bin import CVecEnv


class Rg2Env(gym.Env):
    def __init__(self, urdf_path: str, cfgstr: str, seed: int = 0):
        super().__init__()

        self._cenv = CVecEnv(urdf_path, cfgstr)
        self._cenv.setSeed(seed)

        self.num_obs = self._cenv.getObDim()
        self.num_acts = self._cenv.getActionDim()
        self.num_envs = self._cenv.getNumOfEnvs()
        assert self.num_envs == 1, "Only single environment is supported"

        self.observation_space = spaces.Box(
            low=-1e6, high=1e6, shape=(self.num_obs,), dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-10, high=10, shape=(self.num_acts,), dtype=np.float32
        )

        assert self.num_envs == 1, "Only single environment is supported"

    def seed(self, seed):
        self._cenv.setSeed(seed)

    def turn_on_visualization(self):
        self._cenv.turnOnVisualization()

    def turn_off_visualization(self):
        self._cenv.turnOffVisualization()

    def start_video_recording(self, file_name):
        self._cenv.startRecordingVideo(file_name)

    def stop_video_recording(self):
        self._cenv.stopRecordingVideo()

    def step(self, actions: np.ndarray):
        actions = actions.reshape(1, self.num_acts)
        _reward = np.zeros(1, dtype=np.float32)
        _done = np.zeros(1, dtype=np.bool_)
        self._cenv.step(actions, _reward, _done)
        _observation = np.zeros([1, self.num_obs], dtype=np.float32)

        self._cenv.observe(_observation, False)

        return (_observation.flatten(), _reward.item(), _done.item(), {})

    def reset(self):
        self._cenv.reset()
        _observation = np.zeros([1, self.num_obs], dtype=np.float32)

        self._cenv.observe(_observation, False)

        return _observation[0]

    def close(self):
        self._cenv.close()


from ..utils import RgConfig


ANYMAL_CFG =  """
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

class WebRgEnv(Rg2Env):

    urdf_cfg_pairs = {
        "anymal": (
            "https://raw.githubusercontent.com/raisimTech/raisimLib/master/rsc/anymal/urdf/anymal.urdf",
           ANYMAL_CFG,
        )
    }

    def __init__(self, env_id: str = "anymal"):

        # make dir to save urdf and cfg
        os.makedirs("./gym", exist_ok=True)

        # download urdf and cfg
        import requests

        urdf, cfg = self.urdf_cfg_pairs[env_id]
        with open(f"./gym/{env_id}.urdf", "w") as f:
            f.write(requests.get(urdf).text)
        with open(f"./gym/{env_id}.yaml", "w") as f:
            f.write(cfg)

        # init
        super().__init__(
            f"./gym/{env_id}.urdf", RgConfig.from_yaml(f"./gym/{env_id}.yaml").as_yaml_str()
        )
