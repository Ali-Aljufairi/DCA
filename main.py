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

    def save_image_to_hdf5(self, data_group, step, target_position, image_data):
        dataset_name = f"image_{step}"
        data_group.create_dataset(dataset_name, data=image_data)
        img_dataset = data_group[dataset_name]
        img_dataset.attrs["distance"] = target_position
        img_dataset.attrs["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")

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

        # Calculate the required parameters
        self.calculate_velocity(fps)
        accel_d = self.calculate_accel_distance()
        print(f"accel_d: {accel_d}, type: {type(accel_d)}")

       

        print(f"Moving to position 0 - accel_d...")
        self.move_epics_motor(0 - float(accel_d))
        print("Starting the scan...")
        print(f"Accelerating to steady speed...")
        self.move_epics_motor(self.total_distance + float(accel_d))

            # Steady speed
        print("Acquiring data at steady speed...")

            # Call the start_processes function to handle ZeroMQ communication and data acquisition
        self.start_processes(zmq_port, zmq_host)

        print("Scan completed.")
        print(f"Saved data in {self.hdf_file}")

    def reshape_and_save_image_chunk(self, data_chunk, chunk_idx, stepscan_obj):
        data = np.frombuffer(data_chunk, dtype=np.uint8)
        data = data.reshape(
            (stepscan_obj.image_size_y, stepscan_obj.image_size_x))
        return data

    def reshape_and_save_worker(self, client_id, data_chunk, chunk_idx, stepscan_obj, data_group):
        reshaped_data = self.reshape_and_save_image_chunk(
            data_chunk, chunk_idx, stepscan_obj)
        with data_group.get_lock():
            stepscan_obj.save_image_to_hdf5(
                data_group, chunk_idx, stepscan_obj.motion_stage.position, reshaped_data)
        print(
            f"Client {client_id} reshaped and saved data for chunk {chunk_idx}")

    def client_worker(self, client_id, data_chunks, stepscan_obj, data_group, sync_event):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:1234")

        sync_event.wait()  # Wait for the synchronization event from the server
        sync_event.clear()  # Clear the event for future synchronization

        data = socket.recv()

        # write data to txt file for debugging
        with open(f"client_{client_id}.txt", "wb") as f:
            f.write(data)

        chunk_size = stepscan_obj.image_size_x * stepscan_obj.image_size_y
        num_chunks = 2448*2048 // chunk_size

        for chunk_idx in range(num_chunks):
            chunk_start = chunk_idx * chunk_size
            chunk_end = chunk_start + chunk_size
            data_chunk = data[chunk_start:chunk_end]
            self.reshape_and_save_worker(
                client_id, data_chunk, chunk_idx, stepscan_obj, data_group)

    # function to get the zmq data from the server after image acquisition to be passe to start_processes

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


    def receive_zmq_data(self, zmq_port, zmq_host, chunk_queue):
        context = zmq.Context()
        socket = context.socket(zmq.PULL)
        socket.connect(f"tcp://{zmq_host}:{zmq_port}")

        while True:
            data_chunk = socket.recv()
            if data_chunk == b"EOF":  # Assuming EOF indicates end of data
                break
            chunk_queue.put(data_chunk)

    def receive_data(self, data, chunk_queue):
            chunk_size = self.image_size_x * self.image_size_y
            num_chunks = len(data) // chunk_size

            for i in range(num_chunks):
                start = i * chunk_size
                end = start + chunk_size
                chunk = data[start:end]
                chunk_queue.put(chunk)

    def reshape_and_save(self, chunk_queue, data_group):
        chunk_idx = 0
        while True:
            chunk = chunk_queue.get()
            if chunk is None:
                break

            data = np.frombuffer(chunk, dtype=np.uint8)
            image_data = data.reshape((self.config.image_size_y, self.config.image_size_x))

            with data_group.get_lock():
                self.save_image_to_hdf5(data_group, chunk_idx, self.motion_stage.position, image_data)

            print(f"Reshaped and saved chunk {chunk_idx}")
            chunk_idx += 1

        chunk_queue.put(None)  # Signal complete

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
