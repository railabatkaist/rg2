from rg2 import WebRgUEnv

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback


def make_env(rank):
    def __init():
        env = WebRgUEnv("anymal", seed=rank, visualizable=(rank == 0))
        env = Monitor(env)
        return env

    return __init


envs = DummyVecEnv([make_env(i) for i in range(32)])
envs = VecNormalize(envs, norm_obs=True, norm_reward=False, clip_obs=10.0)

envs.env_method("turn_on_visualization", indices=0)

checkpoint_callback = CheckpointCallback(
    save_freq=1_000_000,
    save_path="./logs/",
    name_prefix="rl_model",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model = PPO("MlpPolicy", envs, verbose=1)
model.learn(total_timesteps=100_000_000, callback=checkpoint_callback)
