import numpy as np
import h5py
import argparse
import time
import epics
import zmq
from multiprocessing import Lock, Process, Queue, current_process
from stepscan import StepScan
from config import *


class ContinuousScan:
    def __init__(self, exposure_time, total_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv, image_size_x, image_size_y, image_counter, num_images, acq_mode, start_acq, acq_status, trigger_mode, trigger_source, trigger_software, image_data, exposure_time_pv, frame_rate_pv, accelaration_time_pv, enable_ndarray, enable_ndarray_callbacks, enable_ZMQ_Array, enable_ZMQ_callbacks, zmq_port, zmq_host):
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
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://127.0.0.1:3000")
        self.fps = epics.caget(self.fps_pv)
        self.enable_ndarray = epics.caput(enable_ndarray, 1)
        self.enable_ndarray_callbacks = epics.caput(
            enable_ndarray_callbacks, 1)
        self.enable_ZMQ_Array = epics.caput(enable_ZMQ_Array, 1)
        self.enable_ZMQ_callbacks = epics.caput(enable_ZMQ_callbacks, 1)

    def send_zmq_message(self, message):
        self.socket.send_string(message)

    def scan_worker(self, step, target_position):
        motor = epics.Motor(self.motion_stage_pv)

        # Move to the target position and wait for the motion to complete
        self.move_epics_motor(target_position)

        # Notify that the image is ready for acquisition
        self.send_zmq_message(f"ImageReady {step}")

        # Wait for the acquisition to complete
        self.send_zmq_message(f"AcquireImage {step}")

        # Save the image data to HDF5 file
        with h5py.File(self.hdf_file, "r+") as f:
            data_group = f["scan_data"]["image_data"]
            self.save_image_to_hdf5(data_group, step, target_position)

    def setup_hdf5_file(self):
        # Create or open an HDF5 file to store the data
        with h5py.File(self.hdf_file, "w") as f:
            # Create the root group
            root_group = f.create_group("scan_data")

            # Add metadata to the root group
            root_group.attrs["exposure_time"] = self.exposure_time
            root_group.attrs["total_distance"] = self.total_distance
            root_group.attrs["step_size"] = self.step_size
            root_group.attrs["num_images"] = self.num_images
            root_group.attrs["acq_mode"] = self.acq_mode
            root_group.attrs["trigger_mode"] = self.trigger_mode
            root_group.attrs["trigger_source"] = self.trigger_source

            # Create the image data group
            data_group = root_group.create_group("image_data")

            # Add image size metadata
            data_group.attrs["image_size_x"] = self.image_size_x
            data_group.attrs["image_size_y"] = self.image_size_y

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
        # Move the motor to the desired position
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:
            time.sleep(0.1)

        print(f"Motor moved to position: {position}")

    def save_image_to_hdf5(self, data_group, step, target_position):
        # Wait for the image counter to change, indicating a new image has been acquired
        initial_counter = epics.caget(self.image_counter)

        while True:
            current_counter = epics.caget(self.image_counter)
            if current_counter != initial_counter:
                print(f"Image acquired with counter {current_counter}")
                break

        # Retrieve the image data
        image_data = epics.caget(self.image_data)
        image_data = np.reshape(
            image_data, (self.image_size_y, self.image_size_x))

        # Create dataset
        img_dataset = data_group.create_dataset(
            f"image_{step}", data=image_data)

        # Add metadata
        img_dataset.attrs["distance"] = target_position
        img_dataset.attrs["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")

    def setup_camera(self):
        epics.caput(self.exposure_time_pv, self.exposure_time)
        epics.caput(self.acq_mode, 2)
        epics.caput(self.trigger_mode, 0)
        epics.caput(self.trigger_source, 0)
        epics.caput(self.camera_acq_pv, 0)

    def perform_continuous_scan(self):
        # Connect to the motion stage and get the fps value and setup the camera
        self.setup_camera()
        self.motion_stage = epics.Motor(self.motion_stage_pv)
        fps = epics.caget(self.fps_pv)

        # Calculate the required parameters
        self.calculate_velocity(fps)
        accel_d = self.calculate_accel_distance()
        print(f"accel_d: {accel_d}, type: {type(accel_d)}")

        self.setup_hdf5_file()

        print(f"Moving to position 0 - accel_d...")
        self.move_epics_motor(0 - float(accel_d))
        print("Starting the scan...")
        print(f"Accelerating to steady speed...")
        self.move_epics_motor(self.total_distance + float(accel_d))

        # Steady speed
        print("Acquiring data at steady speed...")
        epics.caput(self.start_acq, 1)

        print(f"Decelerating and moving to position 0...")
        self.move_epics_motor(0 + float(accel_d))


        print("Scan completed.")
        print(f"Saved data in {self.hdf_file}")


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

    # continuous_scan.perform_continuous_scan()

    position = Queue()
    # tasks_that_are_done = Queue()
    processes = []

    # # creating processes
    # for w in range(number_of_processes):
    #     p = Process(target=continuous_scan.perform_continuous_scan(), args=(tasks_to_accomplish, tasks_that_are_done))
    #     processes.append(p)
    #     p.start()

    p = Process(target=continuous_scan.perform_continuous_scan(), args=())
    processes.append(p)
    p.start()
    # completing process
    for p in processes:
        p.join()


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
