import json


class Config:
    def __init__(self, file_path):
        with open(file_path) as json_file:
            config = json.load(json_file)
        
        # Store the config data as a private attribute
        self._config = config

    def __getattr__(self, name):
        # Check if the attribute exists in the config data
        if name in self._config:
            # If it does, create an instance variable and return its value
            setattr(self, name, self._config[name])
            return self._config[name]
        else:
            # If the attribute is not found in the config data, raise an AttributeError
            raise AttributeError(f"'Config' object has no attribute '{name}'")



