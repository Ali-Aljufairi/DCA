import json

class Config:
    def __init__(self, file_path):
        with open(file_path) as json_file:
            config = json.load(json_file)
        
        # Detector configuration
        self.detector_pv = config.get("detector_pv")

        # Motion stage configuration
        self.motion_stage_pv = config.get("motion_stage_pv")

        # Camera configuration
        self.camera_acq_pv = config.get("camera_acq_pv")
        self.image_size_x = config.get("image_size_x")
        self.image_size_y = config.get("image_size_y")
        self.image_counter = config.get("image_counter")
        self.num_images = config.get("num_images")
        self.acq_mode = config.get("acq_mode")
        self.start_acq = config.get("start_acq")
        self.acq_status = config.get("acq_status")
        self.trigger_mode = config.get("trigger_mode")
        self.trigger_source = config.get("trigger_source")
        self.trigger_software = config.get("trigger_software")
        self.image_data = config.get("image_data")
        self.exposure_time_pv = config.get("exposure_time_pv")
        self.frame_rate_pv = config.get("frame_rate_pv")
        self.enable_ndarray = config.get("enable_ndarray")
        self.enable_ndarray_callbacks = config.get("enable_ndarray_callbacks")
        self.enable_ZMQ_Array = config.get("enable_ZMQ_Array")
        self.enable_ZMQ_callbacks = config.get("enable_ZMQ_Array_callbacks")
        self.velocity = config.get("velocity")
        self.max_velocity = config.get("max_velocity")
        self.acceleratio_time = config.get("acceleration_time")


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
enable_ZMQ_Callbacks = config.enable_ZMQ_Callbacks
velocity = config.velocity