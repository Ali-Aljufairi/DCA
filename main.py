import numpy as np
import time
import argparse
import epics
import json
#try2:
class StepScan:
    def __init__(self, exposure_time, overall_distance, step_size, detector_pv, motion_stage_pv):
        self.exposure_time = exposure_time
        self.overall_distance = overall_distance
        self.step_size = step_size
        self.detector = epics.PV(detector_pv)
        self.motion_stage = epics.Motor(motion_stage_pv)

    def move_motor_to_position(self, position):
        self.motion_stage.move(position)
        while not self.motion_stage.done_moving:  # Wait until the motion is done
            time.sleep(0.1)

    def acquire_image(self):
        # Start the acquisition asynchronously
        self.detector.put('Acquire', 1)

        # Wait for the acquisition to complete
        while self.detector.get('AcquireBusy') == 1:
            time.sleep(0.1)

        image_data3 = self.detector.get('ArrayData')
        # Retrieve the image data
        acquire_time= epics.PV('FLIR5:cam5:AcquireTime').get()
        acquire_time2= epics.caget('FLIR5:cam5:AcquireTime')
        image_data = epics.PV('FLIR5:image1:ArrayData').get()
        image_data2 = epics.caget('FLIR5:image1:ArrayData')

        print(f"acquire_time = {acquire_time}")
        print(f"acquire_time2 = {acquire_time2}")
        print(f"image_data.shape = {image_data}")
        print(f"image_data2.shape = {image_data2}")
        print(f"image_data3.shape = {image_data3}")
        return image_data


    def save_image(self, image_data, file_name):
        # Reshape image according to predefined size
        image_size_x = 2448
        image_size_y = 2048
        image_reshaped = np.reshape(image_data, (image_size_y, image_size_x))
        # Save the image as a PNG file
        np.save(file_name, image_reshaped)

    def start_step_scan(self):
        num_steps = int(self.overall_distance / self.step_size)
        with open('data.xdi', 'w') as data_file:
            data_file.write("# Data columns: Position Current Time\n")
            for step in range(num_steps):
                target_position = step * self.step_size
                self.move_motor_to_position(target_position)
                image_data = self.acquire_image()
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                data_file.write(f"{target_position} {self.motion_stage.get('RBV')} {timestamp}\n")


def main(args):
    # Read PV names from the JSON file
    with open(args.config_file) as json_file:
        config = json.load(json_file)
        detector_pv = config.get("detector_pv")
        motion_stage_pv = config.get("motion_stage_pv")

    step_scan = StepScan(
        args.exposure_time,
        args.overall_distance,
        args.step_size,
        detector_pv,
        motion_stage_pv
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
