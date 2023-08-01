import numpy as np
import h5py
import argparse
import json
import time
import epics
import os
import zmq
from PIL import Image
import threading
import multiprocessing
from stepscan import StepScan
from config import *


class ContinuousScan:

    def __init__(self, exposure_time, distance, step_size, fps, num_images, motion_stage_pv, detector_pv, zmq_ip, zmq_port, exposure_time_pv,ennable_ZMQ_Array, enable_ZMQ_Callbacks, enable_ndarray, enable_ndarray_callbacks,accelaratio_time, trigger_mode, velocity):

        self.exposure_time = exposure_time
        self.exposure_time_pv = epics.caput(exposure_time_pv, exposure_time)
        self.distance = distance
        self.step_size = step_size
        self.fps = epics.caget(fps)
        self.num_images = epics.caput(num_images, 20)
        

        # EPICS initialization
        self.motor = epics.Motor(motion_stage_pv)
        self.detector = epics.PV(detector_pv)

        # Set exposure time
        self.detector.put(exposure_time)

        # Epics PVs initialization
        epics.caput(enable_ndarray, 1)
        epics.caput(enable_ndarray_callbacks, 1)
        epics.caput(ennable_ZMQ_Array, 1)
        epics.caput(enable_ZMQ_Callbacks, 1)
        epics.caput(trigger_mode, 2)

        # Initialize ZMQ
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_ip}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
         
        # Initialize HDF5
        self.f = h5py.File('continuous_scan.hdf5', 'w')
        self.detector_group = self.f.create_group('exchange/detector')
        self.data_group = self.f.create_group('exchange/data')

        # Add metadata
        self.detector_group.attrs['exposure_time'] = exposure_time
        self.detector_group.attrs['num_images'] = num_images

        # Start image collection process
        self.process = multiprocessing.Process(target=self.save_images)

        # Move to start
        self.motor.move(0)

        # Start detector acquisition
        self.detector.acquire(1)

        # Start motion
        self.motor.velocity(velocity)
        while self.motor.position < self.distance:
            time.sleep(self.step_size / velocity)

        # Stop acquisition
        self.detector.acquire(0)

    def save_images(self):

        for i in range(self.num_images):

            # Receive image
            image = self.socket.recv()
            image = np.reshape(image, self.image_shape)

            # Save to HDF5
            self.data_group.create_dataset(f'image_{i}', data=image)

    def stop(self):

        # Stop motion
        self.motor.stop()

        # Close HDF5, ZMQ
        self.f.close()
        self.socket.close()
        self.process.terminate()


def main(args):
    Config(args.config_file)
    continues_scan = ContinuousScan(args.exposure_time, args.overall_distance, args.step_size,
                                    frame_rate_pv, num_images, motion_stage_pv, detector_pv, zmq_host, zmq_port, exposure_time_pv)
    continues_scan.start_scan()
    continues_scan.stop()


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
