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

config = Config("config.json")
detector_pv = config.detector_pv
motion_stage_pv = config.motion_stage_pv
camera_acq_pv = config.camera_acq_pv
image_size_x = config.image_size_x
image_size_y = config.image_size_y
image_counter = config.image_counter
num_images = config.num_images
acq_mode = config.acq_mode
start_acq = config.start_acq
acq_status = config.acq_status
trigger_mode = config.trigger_mode
trigger_source = config.trigger_source
trigger_software = config.trigger_software
image_data = config.image_data
exposure_time_pv = config.exposure_time_pv
frame_rate_pv = config.frame_rate_pv
enable_ndarray = config.enable_ndarray
enable_ndarray_callbacks = config.enable_ndarray_callbacks
enable_ZMQ_Array = config.enable_ZMQ_Array
enable_ZMQ_Callbacks = config.enable_ZMQ_callbacks
velocity = config.velocity
zmq_port = config.zmq_port
zmq_host = config.zmq_host

