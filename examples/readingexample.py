import nodes.NodeManager as nodeManager
import nodes.BufferNode as bufferNode
import time

# Assumed message received from buffer involves the a particular stream. In this case ("Arm" stream). In the buffer, it would look like:
# [<Arm>]1235.00[</Arm>]
# And the message would be 1235.00

manager = nodeManager.ComputeManager()

# Node for communication with V5 Brain
buffer_system = bufferNode.BufferSystem(max_buffer_size=500, port='/dev/ttyACM1', rate=115200)
manager.add_compute_node(buffer_system)

manager.start()

arm_messenger = bufferNode.Messenger(buffer_system, stream="Arm", deleteAfterRead=False)

time.sleep(2)
my_message = arm_messenger.read()
print(my_message)