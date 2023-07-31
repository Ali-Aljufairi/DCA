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
from config import Config


# class ContinuousScan:

#     def __init__(self, exposure_time, distance, step_size, fps, num_images, image_shape,motion_stage_pv, detector_pv, zmq_ip, zmq_port):
        
#         self.exposure_time = exposure_time
#         self.distance = distance
#         self.step_size = step_size
#         self.fps = fps
#         self.num_images = num_images
#         self.image_shape = image_shape
        
#         # EPICS initialization
#         self.motor = epics.Motor(motion_stage_pv)
#         self.detector = epics.PV(detector_pv)
        
#         # Set exposure time
#         self.detector.put(exposure_time)
        
#         # Initialize ZMQ
#         context = zmq.Context()
#         self.socket = context.socket(zmq.SUB)
#         self.socket.connect(f"tcp://{zmq_ip}:{zmq_port}")
#         self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
#         # Initialize HDF5
#         self.f = h5py.File('continuous_scan.hdf5', 'w')
#         self.detector_group = self.f.create_group('exchange/detector')
#         self.data_group = self.f.create_group('exchange/data')

#         # Add metadata
#         self.detector_group.attrs['exposure_time'] = exposure_time
#         self.detector_group.attrs['num_images'] = num_images
        
#         # Start image collection process
#         self.process = multiprocessing.Process(target=self.save_images)
#         self.process.start()
        
#     def start_scan(self):
        
#         # Calculate velocity
#         velocity = self.step_size * self.fps
        
#         # Move to start
#         self.motor.move(0)
        
#         # Start detector acquisition
#         self.detector.acquire(1)
        
#         # Start motion
#         self.motor.velocity(velocity)  
#         while self.motor.position < self.distance:
#             time.sleep(self.step_size / velocity)
            
#         # Stop acquisition  
#         self.detector.acquire(0)
          
#     def save_images(self):
        
#         for i in range(self.num_images):
            
#             # Receive image
#             image = self.socket.recv()
#             image = np.reshape(image, self.image_shape)
            
#             # Save to HDF5
#             self.data_group.create_dataset(f'image_{i}', data=image)
            
#     def stop(self):
        
#         # Stop motion
#         self.motor.stop()
        
#         # Close HDF5, ZMQ
#         self.f.close()
#         self.socket.close()
#         self.process.terminate()
        
# # Start scan
# scan.start_scan()  

# # Stop scan
# scan.stop()














def main(args):
    Config(args.config_file)




    







if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Step Scan using FLIR camera and MICOS stage.")
    parser.add_argument("exposure_time", type=float, help="Exposure time for the FLIR camera.")
    parser.add_argument("overall_distance", type=float, help="Overall distance to scan with the MICOS stage.")
    parser.add_argument("step_size", type=float, help="Step size for each scan step.")
    parser.add_argument("--config_file", default="config.json", help="JSON file containing PV names. (Default: config.json)")
    args = parser.parse_args()
    main(args)
