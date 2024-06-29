import pyrealsense2 as rs
import numpy as np
from collections import namedtuple
from functools import partial
import threading
import time
from scipy.spatial.transform import Rotation as R
import calculators.OffsetTransform as offsetCalculator
import nodes.BufferNode as bufferNode
import toolbox
import json
max_initialization_time = 3


class PoseSystem:
    def __init__(self, serial_odom_messenger:bufferNode.Messenger, debugMode=False):
        """
        Initializes configurations for pose estimation
        """
        self.p = None
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        # if only pose stream is enabled, fps is higher (202 vs 30)
        self.cfg.enable_stream(rs.stream.pose)
        self.lock = threading.Lock()
        self.running = False
        self.debugMode = debugMode
        self.OffsetTransforms = []
        self.errorRunOnce = False
        self.pose_sensor = None
        self.wheel_odometer = None
        self.frame_number = 0
        self.profile = None

        serial_odom_messenger.on_message(self.send_wheel_odometry)

    def send_wheel_odometry(self, data:str):
        if not self.running:
            return
        if not self.device:
            return
        if not self.odom_sensor:
            return
        # data = "x z yaw"
        x, z, yaw = map(float, data.split())

        # Convert yaw (angle) to quaternion
        cy = np.cos(yaw * 0.5) # cosine yaw
        sy = np.sin(yaw * 0.5) # sine yaw

        # Create pose data in RealSense format
        wo_data = rs.pose()
        wo_data.translation.x = x  # Lateral movement
        wo_data.translation.y = 0.0  # Assuming no vertical movement
        wo_data.translation.z = z  # Forward/Backward movement
        wo_data.rotation.w = cy
        wo_data.rotation.x = 0.0
        wo_data.rotation.y = sy
        wo_data.rotation.z = 0.0

        # Send the wheel odometry data to the T265
        self.odom_sensor.send_wheel_odometry(0, self.frame_number, wo_data)
        print("Wheel odometry received")

    def start_pipeline(self, lock: threading.Lock = None):
        """
        This starts the streaming pipeline for the tracking camera.
        """
        if self.running:
            return

        self.profile = self.pipeline.start(self.cfg)
        self.device = self.pipeline.get_active_profile().get_device().as_tm2()
        self.pose_sensor = self.device.first_pose_sensor()

        if self.pose_sensor:
            self.wheel_odometer = self.pose_sensor.as_wheel_odometer()
            with open('no_offset_calibration.json', 'r') as f:
                calibration_data = json.load(f)
                calibration_json = json.dumps(calibration_data)
                self.wheel_odometer.load_wheel_odometry_config(bytearray(calibration_json, 'utf-8'))

        self.running = True

        if lock:
            self.lock = lock

        self.allowable_run = True
        self.first_pass = False

        def runner(self):
            while self.allowable_run:
                if self.debugMode:
                    self.__step()
                else:
                    try:
                        self.__step()
                        self.errorRunOnce = False
                    except Exception as e:
                        if not self.errorRunOnce:
                            self.errorRunOnce = True
                            print(f"Thread Pose Estimation: Error occurred - {e}. Restarting thread.")
                        continue
                if not self.first_pass:
                    self.first_pass = True

        self.pipethread = threading.Thread(target=runner, args=[self])
        self.pipethread.daemon = True
        self.pipethread.start()

        t = 0
        while not self.first_pass:
            t += 0.1
            time.sleep(0.1)
            if t >= max_initialization_time:
                break

    def restart_pipeline(self, lock: threading.Lock = None):
        if self.running:
            self.running = False
            self.pipeline.stop()
            self.pipeline.start(self.cfg)
            self.device = self.pipeline.get_active_profile().get_device()
            self.odom_sensor = self.device.first_wheel_odometer()
            self.running = True
        else:
            self.start_pipeline(lock)

    def stop_pipeline(self):
        """
        This stops the streaming pipeline for the tracking camera.
        """
        if not self.running:
            return
        self.running = False

        self.allowable_run = False
        
        # Ensure pipethread exists before joining
        if hasattr(self, 'pipethread'):
            self.pipethread.join()
        
        try:
            self.pipeline.stop()
        except Exception as e:
            print(f"Error stopping pipeline: {e}")

    def register_offset_transform_stream(self, offsetTransform: offsetCalculator.OffsetTransform, messenger: bufferNode.Messenger, max_decimals=4):
        # Check for duplicates
        for existing in self.OffsetTransforms:
            if isinstance(existing[0], offsetCalculator.OffsetTransform) and existing[0] == offsetTransform:
                print("Duplicate offsetTransform detected, not registering.")
                return
        self.OffsetTransforms.append([offsetTransform, messenger, max_decimals])

    def register_self_transform_stream(self, messenger: bufferNode.Messenger, max_decimals=4):
        # Check for duplicates
        for existing in self.OffsetTransforms:
            if existing[0] is self:
                print("Duplicate self transform detected, not registering.")
                return
        self.OffsetTransforms.append([self, messenger, max_decimals])


    def __step(self):
        """
        This steps the system and applies p, or a pose
        """
        frames = self.pipeline.wait_for_frames()
        if not frames:
            return

        pose_frame = frames.get_pose_frame()
        if not pose_frame:
            return

        self.frame_number = pose_frame.get_frame_number()

        pose = pose_frame.get_pose_data()

        if not pose:
            return

        with self.lock:
            self.p = pose
            
        for i in range(len(self.OffsetTransforms)):
            pose_euler = self.OffsetTransforms[i][0].get_position_and_euler()
            self.OffsetTransforms[i][1].send_euler(pose_euler, max_decimals=self.OffsetTransforms[i][2], confidence=pose.tracker_confidence)


    def get_pose(self):
        return self.p
    
    def get_position_and_euler(self):
        p = self.get_pose()
        return toolbox.get_position_and_euler(p)