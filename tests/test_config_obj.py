import requests
import tempfile
import os
import shutil
import unittest
from rg2.utils import RgConfig


class RgConfigTester(unittest.TestCase):
    
    def test_attr(self):
        
        cfg = RgConfig({
            "a": 1,
            "b": "b",
            "c": [1, 2, 3],
        })
        
        self.assertEqual(cfg.a, 1)
        self.assertEqual(cfg.b, "b")
        self.assertEqual(cfg.c, [1, 2, 3])
        
    def test_from_yaml(self):
        
        tmp_dir = tempfile.mkdtemp()
        tmp_yaml_path = os.path.join(tmp_dir, "test_config.yaml")
        with open(tmp_yaml_path, "w") as f:
            f.write("a: 1\nb: b\nc: [1, 2, 3]")
        
        cfg = RgConfig.from_yaml(tmp_yaml_path)
        
        
        self.assertEqual(cfg.a, 1)
        self.assertEqual(cfg.b, "b")
        self.assertEqual(cfg.c, [1, 2, 3])
    