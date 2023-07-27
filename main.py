import numpy as np
import time
import argparse
import epics
import json
import os
import threading
from PIL import Image
class StepScan:
    def __init__(self, exposure_time, overall_distance, step_size, detector_pv, motion_stage_pv,camera_acq_pv, image_size_x, image_size_y, image_counter, num_images, acq_mode, start_acq, acq_status, trigger_mode, trigger_source, trigger_software, image_data):
        self.exposure_time = exposure_time
        self.overall_distance = overall_distance
        self.step_size = step_size
        self.detector = epics.PV(detector_pv)
        self.motion_stage = epics.Motor(motion_stage_pv)
        self.camera_acq_pv = camera_acq_pv
        self.image_size_x = image_size_x
        self.image_size_y = image_size_y
        self.image_counter = image_counter
        self.num_images = num_images
        self.acq_mode = acq_mode
        self.start_acq = start_acq
        self.acq_status = acq_status
        self.trigger_mode = trigger_mode
        self.trigger_source = trigger_source
        self.trigger_software = trigger_software
        self.image_data = image_data


    def move_motor_to_position(self, position):
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:  # Wait until the motion is done
            time.sleep(0.1)

    def acquire_image(self, camera_acq_pv, acq_status, image_data):
        # Function to acquire image asynchronously

        # Set acquisition mode to continuous
        epics.caput(self.acq_mode, 1)
        # Set trigger mode to "On"
        epics.caput(self.trigger_mode, 1)
        # Set trigger source to "0" (internal)
        epics.caput(self.trigger_source, 0)
        # Set trigger software to "1" (trigger)
        epics.caput(self.trigger_software, 1)

        def image_acquisition_thread():
            # Thread function to acquire the image
            epics.caput(camera_acq_pv, 1)
            while epics.caget(acq_status) == 1:
                print("Acquiring image...")
                time.sleep(0.1)

            # Retrieve the image data
            image_data = epics.caget('FLIR5:image1:ArrayData')

            # Save the acquired image with a file name based on the timestamp (if needed)
            # file_name = f"image_{time.time()}.png"
            # self.save_image(image_data, file_name)

        # Create and start the image acquisition thread
        image_thread = threading.Thread(target=image_acquisition_thread)
        image_thread.start()

        # Check whether the image counter changed or not (for demonstration purposes)
        initial_image_counter = epics.caget(self.image_counter)
        time.sleep(0.5)  # Wait for a short time to allow image acquisition to start
        current_image_counter = epics.caget(self.image_counter)

        if current_image_counter != initial_image_counter:
            print("Image acquisition has started.")
        else:
            print("Image acquisition has not started yet.")



    def save_image(self, image_data, file_name,image_size_x,image_size_y):
        # Create the "images" directory if it does not exist
        if not os.path.exists("images"):
            os.makedirs("images")

        # Reshape image according to predefined size
        image_size_x = epics.caget(image_size_x)
        image_size_y = epics.caget(image_size_y) 
        image_reshaped = np.reshape(image_data, (image_size_y, image_size_x))

        # Save the image in the "images" folder as PNG
        file_path = os.path.join("images", file_name.replace("npy", "png"))
        image_pil = Image.fromarray(image_reshaped)
        image_pil.save(file_path)

    def start_step_scan(self):
        num_steps = int(self.overall_distance / self.step_size)
        with open('data.xdi', 'w') as data_file:
            data_file.write("# Data columns: Position Current Time\n")
            for step in range(num_steps):
                target_position = step * self.step_size
                self.move_motor_to_position(target_position)
                print(f"target pos:        {target_position}")
                image_data = self.acquire_image(self.camera_acq_pv,self.acq_status,self.image_data)
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                data_file.write(f"{target_position}    {self.motion_stage.get('RBV')}    {timestamp}\n")
                # Save the acquired image with a file name based on the timestamp
                # file_name = f"image_{timestamp}.npy"
                # self.save_image(image_data, file_name)


def main(args):
    # Read PV names from the JSON file
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