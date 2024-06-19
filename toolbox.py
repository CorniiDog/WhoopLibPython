import subprocess
import os
from scipy.spatial.transform import Rotation as R
from typing import List
import pyrealsense2 as rs

def find_all_indexes(string:str, substring:str) -> List[int]:
    start = 0
    indexes = []
    while True:
        start = string.find(substring, start)
        if start == -1:
            break
        indexes.append(start)
        start += len(substring)  # Move to the next possible start position
    return indexes

def read_messages_from_buffer(buffer:str, start_marker:str, end_marker:str) -> List[str]:
    start_markers = find_all_indexes(buffer, start_marker)
    end_markers = find_all_indexes(buffer, end_marker)

    # Combine and sort the markers
    start_and_end_markers = [(index, start_marker) for index in start_markers] + [(index, end_marker) for index in end_markers]
    start_and_end_markers.sort()

    messages = []

    starter = False
    start_i = -1
    for index, marker in start_and_end_markers:
        if marker == start_marker and not starter:
            starter = True
            start_i = index + len(start_marker)
        elif marker == end_marker and starter:
            starter = False
            messages.append(buffer[start_i: index])
            start_i = -1

    return messages

def get_latest_message_from_buffer(buffer:str, start_marker:str, end_marker:str) -> str:
    messages = read_messages_from_buffer(buffer, start_marker, end_marker)
    if len(messages) == 0:
        return None
    return messages[-1]

def clear_screen():
    """
    This clears the screen
    """
    try:
        # Check if the operating system is Windows
        if os.name == 'nt':
            subprocess.run('cls', check=True, shell=True)
        else:
            subprocess.run('clear', check=True, shell=True)
    except subprocess.CalledProcessError:
        print("Failed to clear the terminal")



def get_position_and_euler(p):
    """
    This grabs a pose, p, and returns a dictionary
    {'position': (x, y, z), 'euler_angles': (pitch, yaw, roll)}
    In meters and radians
    """
    x, y, z = p.translation.x, p.translation.y, p.translation.z

    # Convert quaternion to Euler angles
    rotation = R.from_quat([p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w])
    euler_angles = rotation.as_euler('xyz', degrees=False)  # Convert to degrees if required
    
    return {'position': (x, y, z), 'euler_angles': euler_angles}

def reset_realsense_devices(lookingfor=2) -> bool:
    # Create a context object. This object owns the handles to all connected realsense devices
    context = rs.context()
    
    # Get a list of all connected devices
    devices = context.query_devices()
    
    i = 0
    if not devices:
        print("No Intel RealSense devices were found.")
    else:
        for dev in devices:
            # The hardware_reset() method sends a hardware reset command that forces the device to disconnect and reconnect
            print(f"Resetting device: {dev.get_info(rs.camera_info.serial_number)}")
            dev.hardware_reset()
            i += 1
            
    if i >= lookingfor:
        return 0
    return 1