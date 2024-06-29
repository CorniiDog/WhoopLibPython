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
        self.odom_sensor = None
        self.wheel_odometer = None
        self.frame_number = 0
        self.profile = None

        serial_odom_messenger.on_message(self.send_wheel_odometry)

    def send_wheel_odometry(self, data:str):
        if not self.running:
            return
        if not self.wheel_odometer:
            print("Wheel odometry sensor not initialized.")
            return

        # data = "x z yaw"
        # Assuming x and z are the components of translational velocity along the respective axes
        # and yaw is not needed for translational velocity calculation directly
        delta_x, delta_z, yaw = map(float, data.split())

        # Create a vector for translational velocity
        translational_velocity = rs.vector()
        translational_velocity.x = delta_x  # Velocity along the x-axis
        translational_velocity.y = 0.0  # Assuming no vertical movement velocity
        translational_velocity.z = delta_z  # Velocity along the z-axis

        # Send the wheel odometry data to the T265
        result = self.wheel_odometer.send_wheel_odometry(0, self.frame_number, translational_velocity)
        if self.debugMode:
            if result:
                print("Wheel odometry successfully sent for frame number:", self.frame_number)
            else:
                print("Failed to send wheel odometry.")

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
            with open('t265_calibration.json', 'r') as f:
                chars = []
                for line in f:
                    for c in line:
                        chars.append(ord(c))  # char to uint8
                self.wheel_odometer.load_wheel_odometery_config(chars)

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