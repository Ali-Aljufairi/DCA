import numpy as np
import h5py
import argparse
import time
import epics
import zmq
import multiprocessing
from stepscan import StepScan
from config import *


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
        self.zmq_port = 1234
        self.zmq_host = "localhost"

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

    def perform_continuous_scan(self, zmq_port, zmq_host):
        # Connect to the motion stage and get the fps value and setup the camera
        self.setup_camera()
        self.motion_stage = epics.Motor(self.motion_stage_pv)
        fps = epics.caget(self.fps_pv)
        self.move_epics_motor(300)
        self.calculate_velocity(fps)
        accel_d = self.calculate_accel_distance()
        print(f"accel_d: {accel_d}, type: {type(accel_d)}")
        self.move_epics_motor(0 - float(accel_d))
        print(f"Accelerating to steady speed...")
        self.move_epics_motor(self.total_distance + float(accel_d))








    def start_processes(self, zmq_port, zmq_host):
        # Create a Queue to hold the data chunks
        chunk_queue = multiprocessing.Queue()

        # Start process to receive data from ZMQ and put in queue
        zmq_process = multiprocessing.Process(
            target=self.receive_zmq_data, args=(zmq_port, zmq_host, chunk_queue))
        zmq_process.start()

        # Start process to take chunks from queue and reshape/save
        reshape_process = multiprocessing.Process(
            target=self.reshape_and_save, args=(chunk_queue,))
        reshape_process.start()

        # Join processes to wait for completion
        zmq_process.join()
        reshape_process.join()




def main(args):
    Config(args.config_file)
    continuous_scan = ContinuousScan(
        args.exposure_time,
        args.overall_distance,
        args.step_size,
        detector_pv,
        motion_stage_pv,
        camera_acq_pv,
        image_size_x,
        image_size_y,
        image_counter,
        num_images,
        acq_mode,
        start_acq,
        acq_status,
        trigger_mode,
        trigger_source,
        trigger_software,
        image_data,
        exposure_time_pv,
        frame_rate_pv,
        accelaration_time_pv,
        enable_ndarray,
        enable_ndarray_callbacks,
        enable_ZMQ_Array,
        enable_ZMQ_Callbacks,
        zmq_port,
        zmq_host)

    continuous_scan.setup_camera()

    continuous_scan.perform_continuous_scan(1234, "localhost")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Step Scan using FLIR camera and MICOS stage.")
    parser.add_argument("exposure_time", type=float,
                        help="Exposure time for the FLIR camera.")
    parser.add_argument("overall_distance", type=float,
                        help="Overall distance to scan with the MICOS stage.")
    parser.add_argument("step_size", type=float,
                        help="Step size for each scan step.")
    parser.add_argument("--config_file", default="config.json",
                        help="JSON file containing PV names. (Default: config.json)")
    args = parser.parse_args()
    main(args)
