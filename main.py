import numpy as np
import time
import argparse
import epics
import json
import os
from PIL import Image
import threading

class StepScan:
    def __init__(self, exposure_time, overall_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv,
                image_size_x, image_size_y, image_counter, num_images, acq_mode, start_acq, acq_status,
                trigger_mode, trigger_source, trigger_software, image_data):
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

        # Set the acquisition mode to multiple
        epics.caput(self.acq_mode, 1)

        # Enable the trigger mode to start the acquisition
        epics.caput(self.trigger_mode, 1)
        epics.caput(self.camera_acq_pv, 1)

        # Set the trigger source to 0 (software triggering)
        epics.caput(self.trigger_source, 0)
        epics.caget(self.image_size_x)
        epics.caget(self.image_size_y)

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

    def acquire_image(self, trigger_software, image_counter, image_data):
        # Trigger the software trigger to initiate image acquisition
        epics.caput(trigger_software, 1)

        # Wait for the image counter to change, indicating a new image has been acquired
        initial_counter = epics.caget(image_counter)
        while True:
            time.sleep(0.1)
            current_counter = epics.caget(image_counter)
            if current_counter != initial_counter:
                break

        # Retrieve the image data
        image_data = epics.caget(image_data)
        return image_data


    def start_step_scan(self):
        num_steps = int(self.overall_distance / self.step_size)
        with open('data.xdi', 'w') as data_file:
            data_file.write("SESAME synchrotron-light\n")
            data_file.write("# Data columns: Position Current Time\n")

            for step in range(num_steps):
                target_position = step * self.step_size
                self.move_motor_to_position(target_position)
                print(f"target pos:        {target_position}")
                image_data = self.acquire_image(self.trigger_software, self.image_counter, self.image_data)
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                data_file.write(f"{target_position}    {self.motion_stage.get('RBV')}    {timestamp}\n")
                file_name = f"image_{timestamp}.png"
                self.save_image(image_data, file_name , self.image_size_x, self.image_size_y) 

def main(args):
    with open(args.config_file) as json_file:
        config = json.load(json_file)
        detector_pv = config.get("detector_pv")
        motion_stage_pv = config.get("motion_stage_pv")
        camera_acq_pv = config.get("camera_acq_pv")
        image_size_x = config.get("image_size_x")
        image_size_y = config.get("image_size_y")
        image_counter = config.get("image_counter")
        num_images = config.get("num_images")
        acq_mode = config.get("acq_mode")
        start_acq = config.get("start_acq")
        acq_status = config.get("acq_status")
        trigger_mode = config.get("trigger_mode")
        trigger_source = config.get("trigger_source")
        trigger_software = config.get("trigger_software")
        image_data = config.get("image_data")

    step_scan = StepScan(
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
        image_data
    )
    step_scan.move_motor_to_position(0)  # Move to the home position (position 0)
    step_scan.start_step_scan()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Step Scan using FLIR camera and MICOS stage.")
    parser.add_argument("exposure_time", type=float, help="Exposure time for the FLIR camera.")
    parser.add_argument("overall_distance", type=float, help="Overall distance to scan with the MICOS stage.")
    parser.add_argument("step_size", type=float, help="Step size for each scan step.")
    parser.add_argument("--config_file", default="config.json", help="JSON file containing PV names. (Default: config.json)")
    args = parser.parse_args()
    main(args)
