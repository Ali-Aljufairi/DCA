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
    def __init__(self, exposure_time, overall_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv,
                 image_size_x, image_size_y, image_counter, num_images, acq_mode, start_acq, acq_status,
                 trigger_mode, trigger_source, trigger_software, image_data, exposure_time_pv, frame_rate,
                 acceleration_time_pv):
        self.exposure_time = exposure_time
        self.overall_distance = overall_distance
        self.step_size = step_size
        self.detector = epics.PV(detector_pv)
        self.motion_stage = epics.Motor(motion_stage_pv)
        self.camera_acq_pv = camera_acq_pv
        self.image_size_x = int(epics.caget(image_size_x))
        self.image_size_y = int(epics.caget(image_size_y))
        self.image_counter = image_counter
        self.num_images = num_images
        self.acq_mode = acq_mode
        self.start_acq = start_acq
        self.acq_status = acq_status
        self.trigger_mode = trigger_mode
        self.trigger_source = trigger_source
        self.trigger_software = trigger_software
        self.image_data = image_data
        self.exposure_time_pv = exposure_time_pv
        self.frame_rate = int(epics.caget(frame_rate))

        # Set the exposure time
        epics.caput(self.exposure_time_pv, self.exposure_time)
        # Set the acquisition mode to continuous
        epics.caput(self.acq_mode, 2)

        # Enable the trigger mode to start the acquisition
        epics.caput(self.trigger_mode, 1)
        epics.caput(self.camera_acq_pv, 1)

        # Set the trigger source to 0 (software triggering)
        epics.caput(self.trigger_source, 0)

        # Get the acceleration time from an external source (e.g., EPICS PV)
        self.acceleration_time = int(epics.caget(acceleration_time_pv))

    def move_motor_to_position(self, position):
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:
            time.sleep(0.1)

    def save_image(self, image_data, file_name, image_size_x, image_size_y):
        if not os.path.exists("images"):
            os.makedirs("images")
        image_reshaped = np.reshape(image_data, (image_size_y, image_size_x))
        file_path = os.path.join("images", file_name.replace("npy", "png"))
        image_pil = Image.fromarray(image_reshaped)
        image_pil.save(file_path)
        print(f"Saved image to {file_path}")

    def acquire_image(self, trigger_software, image_counter, image_data, image_size_x, image_size_y, num_steps):
        # Wait for the image counter to change, indicating a new image has been acquired
        initial_counter = epics.caget(image_counter)

        # Trigger the software trigger to initiate image acquisition
        epics.caput(trigger_software, 1)

        while True:
            time.sleep(0.1)
            current_counter = epics.caget(image_counter)
            if current_counter != initial_counter:
                print(f"Image acquired with counter {current_counter}")
                break

        # Retrieve the image data
        image_data = epics.caget(image_data)
        image_data = np.reshape(image_data, (self.image_size_y, self.image_size_x))
        return image_data

    def continuous_scan(self):
        # Calculate time per frame based on the frame rate
        time_per_frame = 1.0 / self.frame_rate

        # Calculate the time needed for the entire scan (including acceleration phase)
        total_distance = self.overall_distance * self.step_size
        total_time = 2 * self.acceleration_time + (total_distance - 2 * self.acceleration_time) / self.frame_rate

        # Start the scan
        f = h5py.File('continuous_scan.hdf5', 'w')
        detector_group = f.create_group('exchange/detector')
        data_group = f.create_group('exchange/data')
        detector_group.attrs['exposure_time'] = self.exposure_time
        detector_group.attrs['image_size_x'] = self.image_size_x
        detector_group.attrs['image_size_y'] = self.image_size_y
        detector_group.attrs['Num_of_image'] = 1  # Continuous scan produces a single dataset
        detector_group.attrs['local_name'] = "SESAME Detector"
        detector_group.attrs['pixel_size'] = 20E-6  # example

        # Initial position (0)
        self.move_motor_to_position(0)
        initial_time = time.time()

        # Perform the continuous scan
        step_distance = self.step_size
        while step_distance <= total_distance:
            # Move the motor to the target position
            self.move_motor_to_position(step_distance)

            # Acquire an image
            image_data = self.acquire_image(self.trigger_software, self.image_counter, self.image_data,
                                            self.image_size_x, self.image_size_y, 1)

            # Save the acquired image to the HDF5 file
            img_dataset = data_group.create_dataset(f'image_{step_distance}', data=image_data)
            img_dataset.attrs['distance'] = step_distance
            img_dataset.attrs['timestamp'] = time.strftime("%Y-%m-%d %H:%M:%S")

            # Increment the step distance
            step_distance += self.step_size

            # Wait to maintain the frame rate
            elapsed_time = time.time() - initial_time
            if elapsed_time < total_time:
                remaining_time = total_time - elapsed_time
                time.sleep(remaining_time)

        # Final move back to the initial position
        self.move_motor_to_position(0)

        # Close the HDF5 file
        f.close()



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
        accelaration_time_pv
        

    )

    continuous_scan.continuous_scan()


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
