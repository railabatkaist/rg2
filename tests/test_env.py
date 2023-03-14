import unittest
import requests
import tempfile
import os
import shutil
        
from rg2 import CVecEnv, VecEnv
from rg2.utils import RgConfig

URDF_SOURCE_URL = "https://raw.githubusercontent.com/raisimTech/raisimLib/master/rsc/anymal/urdf/anymal.urdf"

CFG_YAML_STR = '''
render: True
num_envs: 100
eval_every_n: 200
num_threads: 30
simulation_dt: 0.0025
control_dt: 0.01
max_time: 4.0
action_std: 0.3
reward:
    forwardVel:
        coeff: 0.3
    torque:
        coeff: -4e-5
'''

class VecEnvTester(unittest.TestCase):
    def test_vecenv_initialization(self):
        
        # Save URDF
        tmp_dir = tempfile.mkdtemp()
        tmp_urdf_path = os.path.join(tmp_dir, "anymal.urdf")
        with open(tmp_urdf_path, "w") as f:
            f.write(requests.get(URDF_SOURCE_URL).text)
        print("Saved URDF to: ", tmp_urdf_path)
        # SAVE YAML
        tmp_yaml_path = os.path.join(tmp_dir, "cfg.yaml")
        with open(tmp_yaml_path, "w") as f:
            f.write(CFG_YAML_STR)
        cfg = RgConfig.from_yaml(tmp_yaml_path)
     
        # test
        VecEnv(CVecEnv(tmp_urdf_path, cfg = cfg.as_yaml_str()))
        # TODO : actual test
        pass