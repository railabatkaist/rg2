from rg2.bin._rg2 import NormalSampler, RaisimGymEnv
from rg2.gym import VecEnv

VecEnv(RaisimGymEnv("./", "hj"))
