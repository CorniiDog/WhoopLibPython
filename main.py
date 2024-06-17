import nodes.PoseNode as poseNode
#import nodes.VisionNode as visionNode
import nodes.NodeManager as nodeManager
import nodes.BufferNode as bufferNode
import calculators.OffsetTransform as offsetCalculator
import toolbox

import time, math

def main():
    # Create a manager
    manager = nodeManager.ComputeManager()
    # Node for T265 Pose (the base)
    t265_pose = poseNode.PoseSystem()
    manager.add_compute_node(t265_pose)
    
    # Node for communication with V5 Brain
    buffer_system = bufferNode.BufferSystem(max_buffer_size=256, port_search_name="VEX Robotics V5 Brain", rate=115200)
    manager.add_compute_node(buffer_system)

    # This is for retreiving the pose of the d435i camera as an offset of the T265 transformation (in meters)
    # This of an x y z offset relative to the T265 camera (rigidly attached)
    # If the camera is slightly forward than the T265, then it would be in the -pz direction (meters)
    # If camera is slightly up, it would be in the +py direction (meters)
    # If camera is slightly right, it would be in +px direction (meters)
    # Image of the transformation origins: https://files.readme.io/29080d9-Tracking_Depth_fixture_image.png
    d43i_pose = offsetCalculator.OffsetTransform(t265_pose, px=-16/1000, py=36.40/1000, pz=-4/1000) 

    # This is the pose messenger system for the V5 Brain
    pose_messenger = bufferNode.Messenger(buffer_system, "P")
    
    # This is the pose of the robot. Same rules apply. It is relative to the pose of the T265 camera
    # The center bottom of the robot is 150 millimeters below camera (since the camera is obviously higher than the bottom of the robot)
    # The camera is perfectly centered on the front of the robot, so px doesn't need to be changed, and neither does the rx, ry, rz, and rw values for rotation.
    # The center of mass of the robot is about 196 millimeters behind the camera, so we set pz to 196/1000 to convert to meters.
    robot_pose = offsetCalculator.OffsetTransform(t265_pose, px=0, py=-150/1000, pz=196/1000)
    # Object node
    # Note: Enabling laser (laser projection) may cause interference w/ another robot's Realsense camera. Recommended to stay disabled.
    #vision = visionNode.VisionSystem(d43i_pose, version="yolov5n", confidence_minimum=0.2, enable_laser=False, width=640, height=480, fps=6)
    #manager.add_compute_node(vision)

    toolbox.reset_realsense_devices()
    manager.start()

    # Register the t265_pose to send robot_pose over the pose_messenger stream whenever the t265_pose updates
    # This allows instantenous sending as soon as data is received, therefore reducing delay
    t265_pose.register_offset_transform_stream(offsetTransform=robot_pose, messenger=pose_messenger, max_decimals=3)

    try:
        print("Running")
        while True:

            """
            # Pose of robot
            robot_pose_euler = robot_pose.get_position_and_euler()

            # Printing the x y z pitch yaw roll of the robot
            position, rotation = robot_pose_euler["position"], robot_pose_euler["euler_angles"]
            x, y, z = position[0], position[1], position[2]
            pitch, yaw, roll = rotation[0], rotation[1], rotation[2]
            """

            """
            print()
            print("[Robot Pose]")
            print(f"x: {x:.2f} m | pitch: {pitch:.2f} rad \ny: {y:.2f} m | yaw: {yaw:.2f} rad \nz: {z:.2f} m | roll: {roll:.2f} rad")
            """

            # Pose of D435i (Same call for robot and T265)
            """
            d435pose_euler = d43i_pose.get_position_and_euler()
            d435pos = d435pose_euler["position"]
            d435ix = d435pos[0]
            d435iy = d435pos[1]
            d435iz = d435pos[2]
            """

            """
            objects = vision.get_objects(name_filter=["sports ball"])

            print("[Objects]")
            for i, object in enumerate(objects):
                name = object["name"]
                x = object["x"] # Position in world space
                y = object["y"]
                z = object["z"]
                print(f"Object {i}: x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
                # You can do whatever with this.
            """ 

            #vision.display_results()
            time.sleep(0.01)

    finally: # <-- If CTRL + C
        manager.stop()

if __name__ == "__main__":
    main()
