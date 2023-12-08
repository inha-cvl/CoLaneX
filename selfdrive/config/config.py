import json
import os
import io

class Config(object):
    _instance = None
    def __new__(cls, map):
        if not isinstance(cls._instance, cls):
            cls._instance = object.__new__(cls)

            with io.open(os.path.join(os.path.dirname(__file__), f'config-{map}.json'), 'r', encoding='utf-8') as f:
                config = json.load(f)
                cls._instance.__dict__ = config

        return cls._instance

    def __getitem__(self, key):
        return getattr(self, key)