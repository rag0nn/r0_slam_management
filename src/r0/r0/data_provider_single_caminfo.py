import rclpy
import rclpy.node
from sensor_msgs.msg import CameraInfo    

import time
from .tools.param_load import load_caminfo_params


class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("provider_caminfo")
        self.topic,self.pub_periot,self.pub_queue_size,_,self.frame_id,camerainfo = load_caminfo_params()
        self.pub_caminfo = self.create_publisher(CameraInfo,self.topic,self.pub_queue_size)
        self.static_cam_info = camerainfo

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {self.pub_periot} second periot...")
        self.timer = self.create_timer(self.pub_periot,self.publish_callback)

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()

        # camera info
        self.static_cam_info.header.stamp = timestamp
        self.static_cam_info.header.frame_id = self.frame_id

        # publish
        self.pub_caminfo.publish(self.static_cam_info)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")

def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

