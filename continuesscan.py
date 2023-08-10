import epics
import zmq
import multiprocessing
import numpy as np
import h5py
import time

class ContinuousScan:
    def __init__(self, exposure_time, total_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv, image_size_x, image_size_y, image_counter,  acq_mode, start_acq, acq_status, trigger_mode, trigger_source, trigger_software, image_data, exposure_time_pv, frame_rate_pv, accelaration_time_pv, enable_ndarray, enable_ndarray_callbacks, enable_ZMQ_Array, enable_ZMQ_callbacks, zmq_port, zmq_host, num_images):
        self.exposure_time = exposure_time
        self.total_distance = total_distance
        self.step_size = step_size
        self.num_steps = int(np.ceil(self.total_distance / self.step_size))
        self.hdf_file = "scan_data.hdf5"
        self.exposure_time_pv = exposure_time_pv
        self.motion_stage_pv = motion_stage_pv
        self.fps_pv = frame_rate_pv
        self.camera_acq_pv = camera_acq_pv
        self.image_size_x = int(epics.caget(image_size_x))
        self.image_size_y = int(epics.caget(image_size_y))
        self.image_counter = image_counter
        self.num_images = num_images
        self.image_data = image_data
        self.acq_mode = acq_mode
        self.start_acq = start_acq
        self.acq_status = acq_status
        self.trigger_mode = trigger_mode
        self.trigger_source = trigger_source
        self.trigger_software = trigger_software
        self.acceleration_time = float(epics.caget(accelaration_time_pv))
        self.motion_stage = None
        self.velocity = None
        self.accel_distance = None
        self.deccel_distance = None
        self.constant_distance = None
        self.acceleration_time_pv = accelaration_time_pv
        self.fps = epics.caget(self.fps_pv)
        self.enable_ndarray = epics.caput(enable_ndarray, 1)
        self.enable_ndarray_callbacks = epics.caput(
            enable_ndarray_callbacks, 1)
        self.enable_ZMQ_Array = epics.caput(enable_ZMQ_Array, 1)
        self.enable_ZMQ_callbacks = epics.caput(enable_ZMQ_callbacks, 1)
        self.num_images = int(np.ceil(self.total_distance / self.step_size))
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.bind(f"tcp://127.0.0.1:1234")
        self.queue = multiprocessing.Queue()

    def receive_data_via_zmq(self):
        while True:
            data = self.socket.recv_pyobj()
            if data is None:
                break
            self.queue.put(data)

    def calculate_total_time(self, fps):
        time_per_frame = 1/fps
        self.total_time = time_per_frame * self.total_distance
        return self.total_time

    def calculate_velocity(self, fps):
        self.calculate_total_time(fps)
        print(f"FPS: {fps}")
        self.velocity = self.total_distance / self.total_time
        return float(self.velocity)

    def calculate_accel_distance(self):
        self.calculate_total_time(self.fps)
        self.accel_distance = (self.total_distance *
                               self.acceleration_time) / self.total_time
        self.deccel_distance = self.accel_distance
        return float(self.accel_distance)

    def calculate_constant_distance(self):
        self.calculate_accel_distance()
        self.constant_distance = self.total_distance - \
            (self.accel_distance + self.deccel_distance)
        return float(self.constant_distance)

    def move_epics_motor(self, position):
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:
            time.sleep(0.1)

    def setup_camera(self):
        epics.caput(self.acq_mode, 1)
        epics.caput(self.trigger_mode, 0)
        epics.caput(self.trigger_source, 0)
        epics.caput(self.camera_acq_pv, 0)

    def save_to_hdf5(self, data):
        with h5py.File(self.hdf_file, 'a') as hdf:
            group_name = f'image_{self.image_counter}'
            hdf.create_group(group_name)
            hdf[group_name]['image_data'] = data

    def process_image_data(self):
        while True:
            if self.queue.empty():
                time.sleep(0.1)
                continue
            data = self.queue.get()
            # Save the acquired image data to HDF5
            self.save_to_hdf5(data)

    def perform_continuous_scan(self):
        # Connect to the motion stage and get the fps value and setup the camera
        self.setup_camera()
        self.motion_stage = epics.Motor(self.motion_stage_pv)
        fps = epics.caget(self.fps_pv)
        self.calculate_velocity(fps)
        accel_d = self.calculate_accel_distance()
        print(f"accel_d: {accel_d}, type: {type(accel_d)}")
        self.move_epics_motor(0 - float(accel_d))
        print(f"Accelerating to steady speed...")

        zmq_process = multiprocessing.Process(target=self.receive_data_via_zmq)
        zmq_process.start()

        processing_process = multiprocessing.Process(
            target=self.process_image_data)
        processing_process.start()
        for _ in range(self.num_images):
            epics.caput(self.start_acq, 1)  # Trigger image acquisition
            time.sleep(self.exposure_time)  # Wait for exposure to complete
            # Capture and process the acquired image data
            image_data = self.image_data  # Replace with actual image data retrieval
            self.queue.put(image_data)

        zmq_process.terminate()
        processing_process.terminate()

        self.move_epics_motor(self.total_distance + float(accel_d))
    def __init__(self, exposure_time, total_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv, image_size_x, image_size_y, image_counter,  acq_mode, start_acq, acq_status, trigger_mode, trigger_source, trigger_software, image_data, exposure_time_pv, frame_rate_pv, accelaration_time_pv, enable_ndarray, enable_ndarray_callbacks, enable_ZMQ_Array, enable_ZMQ_callbacks, zmq_port, zmq_host, num_images):
        self.exposure_time = exposure_time
        self.total_distance = total_distance
        self.step_size = step_size
        self.num_steps = int(np.ceil(self.total_distance / self.step_size))
        self.hdf_file = "scan_data.hdf5"
        self.exposure_time_pv = exposure_time_pv
        self.motion_stage_pv = motion_stage_pv
        self.fps_pv = frame_rate_pv
        self.camera_acq_pv = camera_acq_pv
        self.image_size_x = int(epics.caget(image_size_x))
        self.image_size_y = int(epics.caget(image_size_y))
        self.image_counter = image_counter
        self.num_images = num_images
        self.image_data = image_data
        self.acq_mode = acq_mode
        self.start_acq = start_acq
        self.acq_status = acq_status
        self.trigger_mode = trigger_mode
        self.trigger_source = trigger_source
        self.trigger_software = trigger_software
        self.acceleration_time = float(epics.caget(accelaration_time_pv))
        self.motion_stage = None
        self.velocity = None
        self.accel_distance = None
        self.deccel_distance = None
        self.constant_distance = None
        self.acceleration_time_pv = accelaration_time_pv
        self.fps = epics.caget(self.fps_pv)
        self.enable_ndarray = epics.caput(enable_ndarray, 1)
        self.enable_ndarray_callbacks = epics.caput(
            enable_ndarray_callbacks, 1)
        self.enable_ZMQ_Array = epics.caput(enable_ZMQ_Array, 1)
        self.enable_ZMQ_callbacks = epics.caput(enable_ZMQ_callbacks, 1)
        self.num_images = int(np.ceil(self.total_distance / self.step_size))
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.bind(f"tcp://127.0.0.1:1234")
        self.queue = multiprocessing.Queue()

    def receive_data_via_zmq(self):
        while True:
            data = self.socket.recv_pyobj()
            if data is None:
                break
            self.queue.put(data)

    def calculate_total_time(self, fps):
        time_per_frame = 1/fps
        self.total_time = time_per_frame * self.total_distance
        return self.total_time

    def calculate_velocity(self, fps):
        self.calculate_total_time(fps)
        print(f"FPS: {fps}")
        self.velocity = self.total_distance / self.total_time
        return float(self.velocity)

    def calculate_accel_distance(self):
        self.calculate_total_time(self.fps)
        self.accel_distance = (self.total_distance *
                               self.acceleration_time) / self.total_time
        self.deccel_distance = self.accel_distance
        return float(self.accel_distance)

    def calculate_constant_distance(self):
        self.calculate_accel_distance()
        self.constant_distance = self.total_distance - \
            (self.accel_distance + self.deccel_distance)
        return float(self.constant_distance)

    def move_epics_motor(self, position):
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:
            time.sleep(0.1)

    def setup_camera(self):
        epics.caput(self.acq_mode, 1)
        epics.caput(self.trigger_mode, 0)
        epics.caput(self.trigger_source, 0)
        epics.caput(self.camera_acq_pv, 0)

    def save_to_hdf5(self, data):
        with h5py.File(self.hdf_file, 'a') as hdf:
            group_name = f'image_{self.image_counter}'
            hdf.create_group(group_name)
            hdf[group_name]['image_data'] = data

    def process_image_data(self):
        while True:
            if self.queue.empty():
                time.sleep(0.1)
                continue
            data = self.queue.get()
            # Save the acquired image data to HDF5
            self.save_to_hdf5(data)

    def perform_continuous_scan(self):
        # Connect to the motion stage and get the fps value and setup the camera
        self.setup_camera()
        self.motion_stage = epics.Motor(self.motion_stage_pv)
        fps = epics.caget(self.fps_pv)
        self.calculate_velocity(fps)
        accel_d = self.calculate_accel_distance()
        print(f"accel_d: {accel_d}, type: {type(accel_d)}")
        self.move_epics_motor(0 - float(accel_d))
        print(f"Accelerating to steady speed...")

        zmq_process = multiprocessing.Process(target=self.receive_data_via_zmq)
        zmq_process.start()

        processing_process = multiprocessing.Process(
            target=self.process_image_data)
        processing_process.start()
        for _ in range(self.num_images):
            epics.caput(self.start_acq, 1)  # Trigger image acquisition
            time.sleep(self.exposure_time)  # Wait for exposure to complete
            # Capture and process the acquired image data
            image_data = self.image_data  # Replace with actual image data retrieval
            self.queue.put(image_data)

        zmq_process.terminate()
        processing_process.terminate()

        self.move_epics_motor(self.total_distance + float(accel_d))