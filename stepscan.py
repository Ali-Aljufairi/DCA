import time
import epics
import numpy as np
from PIL import Image
import os
import h5py
class StepScan:
    def __init__(self, exposure_time, overall_distance, step_size, detector_pv, motion_stage_pv, camera_acq_pv,
                image_size_x, image_size_y, image_counter, num_images, acq_mode, start_acq, acq_status,
                trigger_mode, trigger_source, trigger_software, image_data ,exposure_time_pv):
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

        # Set the exposure time
        epics.caput(self.exposure_time_pv, self.exposure_time)
        # Set the acquisition mode to multiple
        epics.caput(self.acq_mode, 1)

        # Enable the trigger mode to start the acquisition
        epics.caput(self.trigger_mode, 1)
        epics.caput(self.camera_acq_pv, 1)

        # Set the trigger source to 0 (software triggering)
        epics.caput(self.trigger_source, 0)

        steps_array = np.arange(0, overall_distance + step_size, step_size)
        print(f"{overall_distance} {step_size} type of overall distance: {type(overall_distance)} type of step size: {type(step_size)}")
        print(f"steps array: {steps_array}")
        num_step= len(steps_array) - 1
        print(f"num steps: {num_step}")
        epics.caput(self.num_images, num_step)
        print(f"num images: {epics.caget(self.num_images)}")

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
        image_data = np.reshape(image_data, (image_size_x, image_size_y))
        return image_data
    def start_step_scan(self):

        f = h5py.File('step_scan.hdf5', 'w')
        
        num_steps = int(self.overall_distance / self.step_size)
        # Create detector and data groups
        detector_group = f.create_group('exchange/detector')
        data_group = f.create_group('exchange/data')

        # Add detector metadata
        detector_group.attrs['exposure_time'] = self.exposure_time  
        detector_group.attrs['image_size_x'] = self.image_size_x
        detector_group.attrs['image_size_y'] = self.image_size_y
        detector_group.attrs['Num_of_image'] = num_steps
        detector_group.attrs['local_name'] = "SESAME Detector"
        detector_group.attrs['pixel_size'] = 20E-6 # example
        

        for step in range(num_steps):

            # Move stage and acquire image
            target_position = step * self.step_size
            self.move_motor_to_position(target_position)
            image_data = self.acquire_image(self.trigger_software, self.image_counter, self.image_data ,self.image_size_x, self.image_size_y,num_steps)
            
            # Create dataset
            img_dataset = data_group.create_dataset(f'image_{step}', data=image_data)
            
            # Add metadata 
            img_dataset.attrs['distance'] = target_position
            img_dataset.attrs['timestamp'] = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Add scan metadata
        data_group.attrs['num_images'] = num_steps
        data_group.attrs['step_size'] = self.step_size

        f.close()