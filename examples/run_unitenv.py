from rg2.gym import WebRgUEnv

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback


def make_env(rank):
    def __init():
        env = WebRgUEnv("anymal", seed=rank, visualizable=(rank == 0))
        env = Monitor(env)
        return env

    return __init


envs = DummyVecEnv([make_env(i) for i in range(16)])
envs = VecNormalize(envs, norm_obs=True, norm_reward=True, clip_obs=10.0)

# envs.env_method("turn_on_visualization", indices=0)

checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./logs/",
    name_prefix="rl_model",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model = PPO("MlpPolicy", envs, verbose=1, batch_size=64, learning_rate=1e-4)
# model = SAC.load("logs/rl_model_96000000_steps.zip")
model.learn(total_timesteps=100_000_000, callback=checkpoint_callback)
