import pyrealsense2 as rs
import torch
import cv2
import numpy as np
import threading
import time
import copy
import nodes.PoseNode as pn
import nodes.BufferNode as bn
from scipy.spatial.transform import Rotation as R


max_initialization_time = 3 # seconds

class VisionSystem:
    def __init__(self, version:str="yolov5n", confidence_minimum:float=0.3, enable_laser=False, debugMode=False, width=640, height=480, fps=6):
        """
        Initializes configurations for object detection
        """
        # Check if CUDA is available
        if torch.cuda.is_available():
            self.device = torch.device("cuda")  # Use CUDA if available
            print("CUDA is available. Using GPU for computations.")
        else:
            self.device = torch.device("cpu")  # Use CPU if CUDA is not available
            print("CUDA is not available. Using CPU for computations.")
        
        # Load the YOLOv5 model from PyTorch Hub
        model = torch.hub.load('ultralytics/yolov5', version, pretrained=True)
        self.names = model.names
        self.lock = threading.Lock()

        # Move your model and tensors to the specified device
        self.model = model.to(self.device)

        # Configure depth and color streams from RealSense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.running = False
        self.confidence_min = confidence_minimum
        self.laser_enabled = enable_laser
        self.debugMode = debugMode
        self.errorRunOnce = False

        self.cv2_window_created = False  # Flag to track if OpenCV window is created

        self.messenger:bn.Messenger = None
        self.messenger_max_decimals = 5

    def register_update_stream(self, messenger, max_decimals=5):
        self.messenger = messenger
        self.messenger_max_decimals = max_decimals

        
    def start_pipeline(self, lock:threading.Lock=None):
        """
        This starts the streaming pipeline for the tracking camera.
        """
        if self.running:
            return
        self.running = True

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        device = self.profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_sensor.set_option(rs.option.emitter_enabled, 1 if self.laser_enabled else 0)
        
        if lock:
            self.lock = lock

        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.color_intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.extrinsics = self.profile.get_stream(rs.stream.color).get_extrinsics_to(self.profile.get_stream(rs.stream.depth))


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
                            print(f"Vision System: Error occurred - {e}. Restarting thread.")
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
            if t  >= max_initialization_time:
                break
    
    def restart_pipeline(self, lock: threading.Lock = None):
        if self.running:
            self.pipeline.stop()
            self.pipeline.start(self.cfg)
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
        
        if self.cv2_window_created:
            try:
                cv2.destroyAllWindows()
                self.cv2_window_created = False
            except Exception as e:
                print(f"Error destroying OpenCV windows: {e}")

    def __step(self):
        """
        This steps the system and creates self.objects which is the result of the computational assembly. Also outputs self.depth_image and self.color_image
        """
        # Wait for a coherent pair of frames: depth and color

        frames = self.pipeline.wait_for_frames(timeout_ms=999999999)
        if not frames:
            return
        
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return
        
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Perform inference
        results = self.model([color_image[:, :, ::-1]])  # Convert BGR (OpenCV format) to RGB

        # Get bounding boxes
        boxes = results.xyxy[0].cpu().numpy()

        send_str = ""

        objects = []
        for box in boxes:
            x1, y1, x2, y2, conf, cls_id = box
            if conf < self.confidence_min:
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Get distance at the center of the bounding box
            distance = depth_frame.get_distance(cx, cy)
            depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [int(cx), int(cy)], distance)

            x, y, z = depth_point
            y *= -1
            z *= -1

            if self.messenger:
                max_d = self.messenger_max_decimals
                send_str += f"[obj][n]{self.names[int(cls_id)]}[/n][x]{x:.{max_d}f}[/x][y]{-y:.{max_d}f}[/y][z]{-z:.{max_d}f}[/z][d]{distance:.{max_d}f}[/d][c]{conf:.{max_d}f}[/c][/obj]"

            object = {"name": self.names[int(cls_id)], "boundingbox":{"x1":x1, "x2":x2, "y1":y1, "y2":y2}, "boundingboxcenter":{"x":cx, "y":cy}, "localposition":{"lx": x, "ly": -y, "lz": -z}, "distance":distance, "confidence": conf}
            objects.append(object)
        if self.messenger:
            self.messenger.send(send_str) # Send objects over to the V5 Brain

        
        
        # Lock and update variables
        with self.lock:
            self.__depth_image = depth_image
            self.__color_image = color_image
            self.__objects = objects
            self.__results = results

    def get_results(self):
        with self.lock:
            a = self.__results
        return a
    
    def get_objects(self, name_filter = []):
        with self.lock:
            if not hasattr(self, "__objects"):
                return []

            if len(name_filter) == 0:
                return copy.deepcopy(self.__objects)
            objects = copy.deepcopy(self.__objects)

        filtered_list = []
        for object in objects:
            for filter in name_filter:
                if object["name"] == filter:
                    filtered_list.append(object)
        return filtered_list
        
    def get_depth_image(self):
        with self.lock:
            return copy.deepcopy(self.__depth_image)
        
    def get_color_image(self):
        with self.lock:
            return copy.deepcopy(self.__color_image)

    def display_results(self, display_inches=False):
        """
        This opens up a display of the results of what is seen
        """
        # Render results
        rendered_frame = np.squeeze(self.get_results().render())
        rendered_frame_bgr = cv2.cvtColor(rendered_frame, cv2.COLOR_RGB2BGR)

        for object in self.get_objects():
            x = object["x"]
            y = object["y"]
            z = object["z"]
            unit = "m"
            if display_inches:
                x *= 39.3701
                y *= 39.3701
                z *= 39.3701
                unit = "in"

            # Draw bounding box and distance
            cv2.rectangle(rendered_frame_bgr, (int(object["boundingbox"]["x1"]), int(object["boundingbox"]["y1"])), (int(object["boundingbox"]["x2"]), int(object["boundingbox"]["y2"])), (0, 255, 0), 2)
            cv2.putText(rendered_frame_bgr, f'x:{x:.2f} y:{y:.2f} z:{z:.2f} {unit}', (object["boundingboxcenter"]["x"], object["boundingboxcenter"]["y"]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Show results
        cv2.imshow('YOLOv5 Detection', rendered_frame_bgr)
        cv2.waitKey(1)
        self.cv2_window_created = True  # Set flag when a window is created
            
