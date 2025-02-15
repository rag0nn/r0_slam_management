import rclpy
import rclpy.node
from sensor_msgs.msg import CameraInfo
import time

from .tools.param_load import load_caminfo_params
MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("listener_caminfo")
        self.topic,_,_,self.listen_periot,_,_ = load_caminfo_params()
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.topic,
            self.listen_callback,
            self.listen_periot
        )

        self.counter_seq = 0

        self.get_logger().info(f"Camera info listening with {self.listen_periot} listen periot...")


    def listen_callback(self,msg):

        self.get_logger().info(f"\n\nDATA: {self.counter_seq} stamp: {msg.header.stamp}")
        self.get_logger().info(f"Received CameraInfo message:")
        self.get_logger().info(f"Width: {msg.width}, Height: {msg.height}")
        self.get_logger().info(f"Distortion Model: {msg.distortion_model}")
        self.get_logger().info(f"Distortion Coefficients: {msg.d}")
        self.get_logger().info(f"Intrinsic Matrix (K): {msg.k}")
        self.get_logger().info(f"Rectification Matrix (R): {msg.r}")
        self.get_logger().info(f"Projection Matrix (P): {msg.p}")
        self.counter_seq += 1


        if self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 0:
            if self.start_time is not None:
                end_time = time.time()
                mean_time = (end_time - self.start_time) / MEAN_PASSED_TIME_FRAME_COUNT
                self.get_logger().info(f"MEAN PASSED TIME PER DATA: {mean_time:.6f} seconds")
            else:
                self.get_logger().warning("Start time is None, skipping mean time calculation.")
        elif self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 1:
            self.start_time = time.time()

def main():
    rclpy.init()
    node = DataListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()