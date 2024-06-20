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
max_initialization_time = 3


class PoseSystem:
    def __init__(self, debugMode=False):
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

    def start_pipeline(self, lock: threading.Lock = None):
        """
        This starts the streaming pipeline for the tracking camera.
        """
        if self.running:
            return
        self.running = True

        self.pipeline.start(self.cfg)

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

        pose = pose_frame.get_pose_data()

        if not pose:
            return

        with self.lock:
            self.p = pose
            
        for i in range(len(self.OffsetTransforms)):
            pose_euler = self.OffsetTransforms[i][0].get_position_and_euler()
            self.OffsetTransforms[i][1].send_euler(pose_euler, max_decimals=self.OffsetTransforms[i][2])


    def get_pose(self):
        return self.p
    
    def get_position_and_euler(self):
        p = self.get_pose()
        return toolbox.get_position_and_euler(p)