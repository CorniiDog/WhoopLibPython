import threading

class ComputeManager:
    def __init__(self):
        self.thread_lock = threading.Lock() # Create a thread lock to reduce memory issues
        self.computes = []

    def add_compute_node(self, node):
        self.computes.append(node)

    def start(self):
        for compute in self.computes:
            compute.start_pipeline(self.thread_lock)

    def stop(self):
        for compute in self.computes:
            compute.stop_pipeline()

    