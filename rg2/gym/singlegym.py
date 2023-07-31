# Author : Simo Ryu
from typing import Optional, List

import numpy as np
import platform
import os

import gym
from gym import spaces
from ..bin import CWalkerEnv, CWalkerEnvConfig
from ..utils import RgConfig
import requests
import zipfile
import numpy as np
import dataclasses


@dataclasses.dataclass
class WalkerEnvConfig:
    resource_dir: str
    gc_init: np.ndarray
    gv_init: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray
    p_gain: float
    d_gain: float
    env_params: list

    def __post_init__(self):
        self.gc_init = np.array(self.gc_init, dtype=np.float64)
        self.gv_init = np.array(self.gv_init, dtype=np.float64)
        self.action_mean = np.array(self.action_mean, dtype=np.float64)
        self.action_std = np.array(self.action_std, dtype=np.float64)
        self.env_params = list(self.env_params)

    def get_cpp_object(self):
        config_dict = {
            "resourceDir": self.resource_dir,
            "gcInit": self.gc_init,
            "gvInit": self.gv_init,
            "actionMean": self.action_mean,
            "actionStd": self.action_std,
            "pGain": self.p_gain,
            "dGain": self.d_gain,
            "envParams": self.env_params,
        }
        return CWalkerEnvConfig(config_dict)


ANYMAL_NOMINAL_STATE = [
    0,
    0,
    0.50,
    1.0,
    0.0,
    0.0,
    0.0,
    0.03,
    0.4,
    -0.8,
    -0.03,
    0.4,
    -0.8,
    0.03,
    -0.4,
    0.8,
    -0.03,
    -0.4,
    0.8,
]


class Rg2UEnv(gym.Env):
    def __init__(
        self, config: WalkerEnvConfig, seed: int = 0, visualizable: bool = False
    ):
        super().__init__()

        self._cenv = CWalkerEnv(config, visualizable)
        self._cenv.setSeed(seed)

        self.num_obs = self._cenv.getObDim()

        self.num_acts = self._cenv.getActionDim()

        self.observation_space = spaces.Box(
            low=-1e6, high=1e6, shape=(self.num_obs,), dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-10, high=10, shape=(self.num_acts,), dtype=np.float32
        )

        self._cenv.setControlTimeStep(0.01)
        self._cenv.setSimulationTimeStep(0.0025)
        self._cenv.reset()

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

        return _observation.flatten()

    def close(self):
        self._cenv.close()


class WebRgUEnv(Rg2UEnv):
    urdf_cfg_pairs = {
        "anymal": (
            "https://github.com/railabatkaist/rg2/blob/main/examples/rsc/anymal.zip?raw=true",
            "urdf/anymal.urdf",
        )
    }

    def __init__(
        self,
        env_id: str = "anymal",
        seed: int = -1,
        visualizable: bool = False,
        env_params: Optional[List[float]] = [-1, -1, -1, -1, -1, -1],
    ) -> None:
        # make dir to save urdf and cfg
        os.makedirs("./gym", exist_ok=True)
        os.makedirs(f"./gym/{env_id}", exist_ok=True)

        # download urdf and cfg

        urdf, entry_point = self.urdf_cfg_pairs[env_id]

        with open(f"./gym/{env_id}/file.zip", "wb") as f:
            f.write(requests.get(urdf).content)

        # unzip urdf
        with zipfile.ZipFile(f"./gym/{env_id}/file.zip", "r") as zip_ref:
            zip_ref.extractall(f"./gym/{env_id}")

        cfg = WalkerEnvConfig(
            resource_dir=str(os.path.abspath(f"./gym/{env_id}/{env_id}/{entry_point}")),
            gc_init=ANYMAL_NOMINAL_STATE,
            gv_init=[0] * (len(ANYMAL_NOMINAL_STATE) - 1),
            action_mean=ANYMAL_NOMINAL_STATE[-12:],
            action_std=[0.3] * 12,
            p_gain=50,
            d_gain=0.2,
            env_params=env_params,
        ).get_cpp_object()

        super().__init__(
            cfg,
            seed=seed,
            visualizable=visualizable,
        )
