import numpy as np
import platform
import os

import gym
from gym import spaces
from ..bin import CVecEnv


class Rg2Env(gym.Env):
    def __init__(self, urdf_path: str, cfg: str, seed: int = 0):
        super().__init__()

        self._cenv = CVecEnv(urdf_path, cfg)
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
        _reward = np.zeros(1, dtype=np.float32)
        _done = np.zeros(1, dtype=np.bool_)
        self._cenv.step(actions, _reward, _done)

        _observation = np.zeros([1, self.num_obs], dtype=np.float32)

        self._cenv.observe(_observation, False)

        return (_observation[0], _reward[0], _done[0], {})

    def reset(self):
        self._cenv.reset()
        _observation = np.zeros([1, self.num_obs], dtype=np.float32)

        self._cenv.observe(_observation, False)

        return _observation[0]
    
    def close(self):
        self._cenv.close()
    
        