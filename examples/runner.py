from rg2 import Rg2Env
from rg2.utils import RgConfig
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env


cfgstr = RgConfig.from_yaml(tmp_yaml_path).as_yaml_str()
env = Rg2Env(tmp_urdf_path, cfgstr=cfgstr)

envs = make_vec_env(
    Rg2Env, n_envs=8, env_kwargs={"urdf_path": tmp_urdf_path, "cfgstr": cfgstr}
)

model = PPO("MlpPolicy", envs, verbose=1)
model.learn(total_timesteps=10_0000)
