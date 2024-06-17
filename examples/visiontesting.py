import pyrealsense2 as rs
import torch
import cv2
import numpy as np
import toolbox

display_view = False # Displays what the camera sees on the desktop (can't be ssh'd)
print_results = True # Print in output 

print("Torch version:", torch.__version__)

# Check if CUDA is available
if torch.cuda.is_available():
    device = torch.device("cuda")  # Use CUDA if available
    print("CUDA is available. Using GPU for computations.")
else:
    device = torch.device("cpu")  # Use CPU if CUDA is not available
    print("CUDA is not available. Using CPU for computations.")

# Load the YOLOv5 model from PyTorch Hub
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

# Move your model and tensors to the specified device
model = model.to(device)
names = model.names

toolbox.reset_realsense_devices()

# Configure depth and color streams from RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)

# Start streaming
profile = pipeline.start(config)

depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
extrinsics = profile.get_stream(rs.stream.color).get_extrinsics_to(profile.get_stream(rs.stream.depth))

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames(timeout_ms=999999999)
        if not frames:
            continue
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Perform inference
        results = model([color_image[:, :, ::-1]])  # Convert BGR (OpenCV format) to RGB

        # Render results
        rendered_frame = np.squeeze(results.render())
        rendered_frame_bgr = cv2.cvtColor(rendered_frame, cv2.COLOR_RGB2BGR)

        # Get bounding boxes
        boxes = results.xyxy[0].cpu().numpy()

        for box in boxes:
            x1, y1, x2, y2, conf, cls_id = box
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Get distance at the center of the bounding box
            distance = depth_frame.get_distance(cx, cy)

            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [int(cx), int(cy)], distance)

            x, y, z = depth_point

            if print_results:
                print("Object:", names[int(cls_id)], x, y, z)

            if display_view:
                # Draw bounding box and distance
                cv2.rectangle(rendered_frame_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(rendered_frame_bgr, f'{x:.2f} {y:.2f} {z:.2f} m', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if display_view:
            # Show results
            cv2.imshow('YOLOv5 Detection', rendered_frame_bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                break

finally:
    # Stop streaming
    pipeline.stop()

    