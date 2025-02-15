import rclpy
import rclpy.node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from glob import glob
import time
from .tools.param_load import load_rgb_params

class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("provider_rgb")
        self.topic ,self.pub_periot ,self.pub_queue_size ,self.listen_periot ,self.frame_id,self.data_path, self.typeruntime,self.typeshowtime = load_rgb_params()
        print(self.data_path)
        self.rgb_names = []
        self._get_image_names()

        self.bridge = CvBridge()

        self.pub_rgb = self.create_publisher(Image,self.topic,self.pub_queue_size)

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {self.pub_periot} second periot...")
        self.timer = self.create_timer(self.pub_periot,self.publish_callback)

            
    def sequence_read_callback(self):
        frame = cv2.imread(self.rgb_names[self.counter_seq])
        print(f"\nIMAGE RGB: {self.rgb_names[self.counter_seq]} SHAPE: ",frame.shape)
        self.counter_seq +=1
        return frame

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        rgb = self.sequence_read_callback()

        # rgb
        msg_rgb = self.bridge.cv2_to_imgmsg(rgb, encoding= self.typeruntime)
        msg_rgb.header.stamp = timestamp
        msg_rgb.header.frame_id = self.frame_id
        
        # publish
        self.pub_rgb.publish(msg_rgb)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")

    def _get_image_names(self):
        rgb_names = sorted(glob(f"{self.data_path}/*"))        
        self.rgb_names = rgb_names



def main():
    
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

