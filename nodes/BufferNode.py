import serial.tools
import serial.tools.list_ports
import toolbox
import serial
import time
import threading
from typing import Dict, List

max_initialization_time = 3


class BufferSystem:
    def __init__(self, max_buffer_size=512, port_search_name="VEX Robotics V5 Brain", rate=115200, debugMode=False):
        """
        max_buffer_size is the max number of characters to record. 500 is default.
        """
        self.registered_messengers = []
        self.messages = {}
        self.buffer = ""
        self.lock = threading.Lock()
        self.running = False
        self.allowable_run = True
        self.max_buffer_size = max_buffer_size
        self.port = None
        self.rate = rate
        self.ser = None
        self.debugMode= debugMode
        self.port_search_name = port_search_name
        self.errorRunOnce = False

    def find_vex_robotics_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if self.port_search_name in port.description:
                return port.device
        return None

    def register_stream(self, messenger):
        """
        This registers a messenger object.
        """
        for i in range(len(self.registered_messengers)):
            msgr = self.registered_messengers[i]
            if messenger.stream == msgr.stream:
                raise Exception(f"Stream {messenger.stream} is already registered. Configure a different name.")
        self.registered_messengers.append(messenger)

    def start_pipeline(self, lock: threading.Lock = None):
        """
        This starts the streaming pipeline for the serial connection reading.
        """
        if lock:
            self.lock = lock

        if self.running:
            return
        self.running = True
        self.first_pass = False

        def runner():
            self.allowable_run = True

            while self.allowable_run:
                if self.debugMode:
                    self.__step()
                else:
                    try:
                        self.__step()
                    except Exception as e:
                        print(f"Thread Buffer Estimation: Error occurred - {e}. Restarting thread.")
                        continue
                if not self.first_pass:
                    self.first_pass = True
                time.sleep(0.01)  # Add a small delay to prevent high CPU usage

        self.pipethread = threading.Thread(target=runner)
        self.pipethread.daemon = True  # Ensure the thread will exit when the main program exits
        self.pipethread.start()

        t = 0
        while not self.first_pass:
            t += 0.1
            time.sleep(0.1)
            if t >= max_initialization_time:
                break
    
    def reconnect(self):
        try:
            port = self.find_vex_robotics_port()
            if not port:
                return 1
            if self.ser:
                if self.ser.isOpen():
                    self.ser.close()  # Ensure the connection is closed before reopening
                self.ser = None  # Reset the serial object
            self.port = port
            self.ser = serial.Serial(self.port, self.rate, timeout=1)
            return 0
        except serial.SerialException as e:
            print("Error attempting to reconnect:", e)
            return 1

    def __step(self):

        def run():
            try:
                if not self.ser: # Initialize
                    if self.reconnect() != 0:
                        if not self.errorRunOnce:
                            self.errorRunOnce = True
                            print("Reconnect error, trying again")
                        time.sleep(0.5)
                        return

                if self.ser.in_waiting > 0:  # Check if there's data in the buffer
                    data = self.ser.read(self.ser.in_waiting)  # Read all available data in the buffer
                    if data:
                        buffer_stream = data.decode('utf-8').strip()
                        #print("Read message:", system_messages)
                        with self.lock:
                            self.buffer += buffer_stream
                            # Ensure the buffer doesn't exceed max_buffer_size
                            if len(self.buffer) > self.max_buffer_size:
                                self.buffer = self.buffer[-self.max_buffer_size:]
                        
                        for messenger in self.registered_messengers:
                            latest_msg = toolbox.get_latest_message_from_buffer(self.buffer, "[<" + messenger.stream + ">]", "&=" + messenger.stream + "*$")
                            if latest_msg:
                                with self.lock:
                                    self.messages[messenger.stream] = latest_msg.strip()
                                for i in range(len(messenger.callback_functions)):
                                    messenger.callback_functions[i](latest_msg.strip())
                if self.errorRunOnce:
                    print("Error resolved")
                    self.errorRunOnce = False

            except serial.SerialException as e:
                print("Serial communication error:", e)
                if self.reconnect() != 0:
                    if not self.errorRunOnce:
                            self.errorRunOnce = True
                            print("Reconnect error, trying again")
                    time.sleep(0.5)
                    return
                time.sleep(0.5)
        
        if self.debugMode:
            run()
        else:
            try:
                run()
            except Exception as e:
                if self.reconnect() != 0:
                    if not self.errorRunOnce:
                            self.errorRunOnce = True
                            print("Reconnect error, trying again")
                    time.sleep(0.5)
                    return
                print("Runtime Error:", e)
                time.sleep(2)


    def stop_pipeline(self):
        """
        This stops the streaming pipeline for the serial connection reading.
        """
        if not self.running:
            return
        self.running = False
        self.allowable_run = False
        
        # Ensure pipethread exists before joining
        if hasattr(self, 'pipethread'):
            self.pipethread.join()
        
        try:
            if self.ser and self.ser.isOpen():
                self.ser.close()
        except Exception as e:
            print(f"Error stopping pipeline: {e}")

    def get_message(self, stream:str, delete_after_read=False) -> str:
        """
        Retreives message from robot, as a string, from the stream.
        """
        if stream in self.messages:
            with self.lock:
                if delete_after_read:
                    message = self.messages[stream]
                    del self.messages[stream]
                    return message

                return self.messages[stream]
        else:
            return None
        
    def send_message(self, stream:str, message:str, end="\n") -> bool:
        """
        Sends a message to the dedicated stream
        """
        def send():
            if not self.ser: # Initialize
                if self.reconnect() != 0:
                    if not self.errorRunOnce:
                        self.errorRunOnce = True
                        print("Reconnect error")
                    return

            box = f"[<{stream}>]{message}&={stream}*${end}".encode('utf-8')
            with self.lock: # Write protection for sending (to avoid another serial writing utility from weaving additional data into the packets)
                self.ser.write(box)  # Encode string to bytes and send it over serial
            if self.errorRunOnce:
                    print("Error resolved")
                    self.errorRunOnce = False
        if self.debugMode:
            send()
            return True
        else:
            try:
                send()
                return True
            except:
                if self.reconnect() != 0:
                    if not self.errorRunOnce:
                        self.errorRunOnce = True
                        print("Reconnect error")
                return False
        

class Messenger:
    def __init__(self, bufferSystem:BufferSystem, stream:str, deleteAfterRead=False):
        """
        Requires creating a BufferSystem object to create, and a stream that is synced from the robot.
        """
        self.stream = stream
        self.bufferSystem = bufferSystem
        self.bufferSystem.register_stream(stream)
        self.deleteAfterRead = deleteAfterRead
        self.callback_functions = []

    def send(self, message:str) -> bool:
        """
        Sends a message to the robot.
        """
        return self.bufferSystem.send_message(self.stream, message=message)

    def send_euler(self, pose_euler:Dict, max_decimals:int=-1) -> bool:
        """
        Sends a euler dict to the robot.
        """
        position, rotation = pose_euler["position"], pose_euler["euler_angles"]
        x, y, z = position[0], position[1], position[2]
        pitch, yaw, roll = rotation[0], rotation[1], rotation[2]

        if max_decimals >= 0:
            self.send(f"{x:.{max_decimals}f} {y:.{max_decimals}f} {z:.{max_decimals}f} {pitch:.{max_decimals}f} {yaw:.{max_decimals}f} {roll:.{max_decimals}f}")
        else:
            self.send(f"{x} {y} {z} {pitch} {yaw} {roll}")

    def read(self) -> str:
        """
        Attempts to read a message from the robot.
        """
        return self.bufferSystem.get_message(self.stream, delete_after_read=self.deleteAfterRead)
    
    def read_euler(self) -> Dict:
        """
        Reads a euler dict from the robot.
        """
        message = self.read()
        if not message:
            return None
        
        sub_messages = message.strip().split(" ")
        if len(sub_messages) != 6:
            return None
        
        return {"position": [float(sub_messages[0]), float(sub_messages[1]), float(sub_messages[2])], 
                "euler_angles":[float(sub_messages[3]), float(sub_messages[4]), float(sub_messages[5])]}

    def on_message(self, callback_function):
        self.callback_functions.append(callback_function)

if __name__ == "__main__":
    reader = BufferSystem()
    reader.register_read_stream("Arm")
    reader.start_pipeline()

    # Allow some time to collect data
    time.sleep(2.01)

    print(reader.get_message("Arm", delete_after_read=True))
    print(reader.get_message("Arm", delete_after_read=True))