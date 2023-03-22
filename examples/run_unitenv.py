from rg2 import WebRgUEnv
from rg2.utils import RgConfig
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

envs = make_vec_env(
    WebRgUEnv, n_envs=8, env_kwargs={"env_id": "anymal"}
)

envs = VecNormalize(envs, norm_obs=True, norm_reward=True, clip_obs=10.0)


model = PPO("MlpPolicy", envs, verbose=1)
model.learn(total_timesteps=100_000_000)
