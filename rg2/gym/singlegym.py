# Author : Simo Ryu

import numpy as np
import platform
import os

import gym
from gym import spaces
from ..bin import UnitEnv
from ..utils import RgConfig
import requests


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
class Rg2UEnv(gym.Env):
    
    def __init__(self, urdf_path: str, cfgstr: str, seed: int = 0, visualizable: bool = False):
        super().__init__()

        self._cenv = UnitEnv(urdf_path, cfgstr, visualizable)
        self._cenv.setSeed(seed)

        self.num_obs = self._cenv.getObDim()
        self.num_acts = self._cenv.getActionDim()
      
        self.observation_space = spaces.Box(
            low=-1e6, high=1e6, shape=(self.num_obs,), dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-10, high=10, shape=(self.num_acts,), dtype=np.float32
        )
        self._cenv.init()
        self._cenv.reset()
        self._cenv.setControlTimeStep(0.01)
        self._cenv.setSimulationTimeStep(0.0025)
    
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
        actions = actions.reshape(self.num_acts)
        
        reward = self._cenv.step(actions)
        terminal_reward = 0.0
        done = self._cenv.isTerminalState(terminal_reward)
        _observation = np.zeros([self.num_obs, 1], dtype=np.float32)

        self._cenv.observe(_observation)
        
    

        return (_observation.flatten(), reward + terminal_reward, done, {})
    
    def reset(self):
        self._cenv.reset()
        _observation = np.zeros([self.num_obs, 1], dtype=np.float32)

        self._cenv.observe(_observation)

        return _observation[0]

    def close(self):
        self._cenv.close()
        

class WebRgUEnv(Rg2UEnv):

    urdf_cfg_pairs = {
        "anymal": (
            "https://raw.githubusercontent.com/raisimTech/raisimLib/master/rsc/anymal/urdf/anymal.urdf",
           ANYMAL_CFG,
        )
    }

    def __init__(self, env_id: str = "anymal", seed: int = -1, visualizable: bool = False):

        # make dir to save urdf and cfg
        os.makedirs("./gym", exist_ok=True)
        os.makedirs(f"./gym/{env_id}", exist_ok=True)

        # download urdf and cfg
        
        urdf, cfg = self.urdf_cfg_pairs[env_id]
        with open(f"./gym/{env_id}/anymal.urdf", "w") as f:
            f.write(requests.get(urdf).text)
        with open(f"./gym/{env_id}/anymal.yaml", "w") as f:
            f.write(cfg)
#"./gym/{env_id}/anymal.urdf"
        # init
        super().__init__(
            f"./gym/{env_id}/anymal.urdf", RgConfig.from_yaml(f"./gym/{env_id}/anymal.yaml").as_yaml_str(), seed = seed , visualizable = visualizable
        )
