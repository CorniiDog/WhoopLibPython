import numpy as np
from scipy.spatial.transform import Rotation as R
import toolbox

class Pose:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class OffsetTransform:
    def __init__(self, compute_node, px:float=0, py:float=0, pz:float=0, rx:float=0, ry:float=0, rz:float=0, rw:float=1):
        self.compute_node = compute_node
        self.offset_translation = np.array([px, py, pz])
        self.offset_rotation = R.from_quat([rx, ry, rz, rw])

    def get_pose(self):
        p = self.compute_node.get_pose()
        
        # Extract the components of p assuming they are structured with attributes
        current_rotation = R.from_quat([p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w])
        current_translation = np.array([p.translation.x, p.translation.y, p.translation.z])
        
        # Rotate the offset translation by the current rotation and add it to the current translation
        rotated_translation = current_rotation.apply(self.offset_translation)
        new_translation = current_translation + rotated_translation
        
        # Combine the rotations
        new_rotation = current_rotation * self.offset_rotation
        new_quaternion = new_rotation.as_quat()
        
        # Create a new Quaternion and Vector for the new pose
        new_pose_rotation = Quaternion(new_quaternion[0], new_quaternion[1], new_quaternion[2], new_quaternion[3])
        new_pose_translation = Vector(new_translation[0], new_translation[1], new_translation[2])
        
        # Return a new Pose object with the updated rotation and translation
        return Pose(new_pose_rotation, new_pose_translation)
        
    def get_position_and_euler(self):
        p = self.get_pose()
        return toolbox.get_position_and_euler(p)

