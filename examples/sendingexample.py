import nodes.NodeManager as nodeManager
import nodes.BufferNode as bufferNode
import time

# Assumed message received from buffer involves the a particular stream. In this case ("Arm" stream). In the buffer, it would look like:
# [<Arm>]1235.00[</Arm>]
# And the message would be 1235.00

manager = nodeManager.ComputeManager()

# Node for communication with V5 Brain
buffer_system = bufferNode.BufferSystem(max_buffer_size=512, port_search_name="VEX Robotics V5 Brain", rate=115200)
manager.add_compute_node(buffer_system)

manager.start()

pose_messenger = bufferNode.Messenger(buffer_system, stream="Pose")

x,y,z = 1, 2, 3
pitch, yaw, roll = 0.1, 0.2, 0.3

while True:
    euler_pose = {"position": [x, y, z], "euler_angles": [pitch, yaw, roll]}
    print("Sending")
    pose_messenger.send_euler(euler_pose, max_decimals=2)
    print("Sent", euler_pose)

    x += 0.1
    y += 0.1
    z += 0.1
    pitch += 0.01
    yaw += 0.01
    roll += 0.01
    time.sleep(1)