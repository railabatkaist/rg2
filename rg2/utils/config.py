
from typing import Optional


from ruamel.yaml import YAML, dump, RoundTripDumper

class RgConfig:
    r"""
    Configuration to be used for various rg2 gym environments.
    It manages loading same configuration format from YAML, gin, json, etc.
    Config hash is calculated from string representation of YAML config.
    
    TODO:
    - [ ] Support JSON, GIN config
    """
    
    _hash = None
    
    def __init__(self, config: Optional[dict]):
        self.config = config
        
    def __hash__(self):
        if self._hash is None:
            self._hash = hash(self.as_yaml_str())
        return self._hash
    
    def __repr__(self):
        return self.as_yaml_str()
    
        
    @classmethod
    def from_yaml(cls, yaml_path : str):
        _config = YAML().load(open(yaml_path))
        return cls(_config)
    
    
    def as_yaml_str(self)->str:
        return dump(self.config, Dumper=RoundTripDumper)
    
    