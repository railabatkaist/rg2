from rg2 import WebRgEnv
from rg2.utils import RgConfig
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

envs = make_vec_env(
    WebRgEnv, n_envs=8, env_kwargs={"env_id": "anymal"}
)

model = PPO("MlpPolicy", envs, verbose=1)
model.learn(total_timesteps=100_000_000)
