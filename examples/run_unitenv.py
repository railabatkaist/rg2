from rg2 import WebRgUEnv
from rg2.utils import RgConfig
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback

envs = make_vec_env(WebRgUEnv, n_envs=8, env_kwargs={"env_id": "anymal"})

envs = VecNormalize(envs, norm_obs=True, norm_reward=False, clip_obs=10.0)


checkpoint_callback = CheckpointCallback(
    save_freq=1000,
    save_path="./logs/",
    name_prefix="rl_model",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model = PPO("MlpPolicy", envs, verbose=1)
model.learn(total_timesteps=100_000_000, callback=checkpoint_callback)
