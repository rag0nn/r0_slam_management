import rclpy
import rclpy.node
from sensor_msgs.msg import Image
import cv2
import numpy as np

from cv_bridge import CvBridge
import time

from .tools.param_load import load_depth_params
MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("listener_depth")
        self.topic ,_,_ ,self.listen_periot ,_,_,self.typeruntime,self.typeshowtime= load_depth_params()

        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            self.topic,
            self.listen_callback,
            self.listen_periot
        )

        self.counter_seq = 0
        self.start_time = None  

        self.get_logger().info(f"Image sequence listening with {self.listen_periot} listen periot...")


    def listen_callback(self,msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.typeshowtime)
            self.get_logger().info(f"\nDATA:{self.counter_seq} stamp: {msg.header.stamp}")
            print(cv_image.shape)
            print(cv_image.dtype)
            print(cv_image.min(),cv_image.max())    
            # cv2 can't show correctly image from 16UC1 because max range is bigger than 255.
            # we are normalizing the image to (0,255) range
            cv_image= cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image = cv_image.astype(np.uint8)

            cv2.imshow("DEPTH Image", cv_image)
            cv2.waitKey(1)  
            self.counter_seq += 1
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

        if self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 0:
            if self.start_time is not None:
                end_time = time.time()
                mean_time = (end_time - self.start_time) / MEAN_PASSED_TIME_FRAME_COUNT
                self.get_logger().info(f"MEAN PASSED TIME PER DATA: {mean_time:.6f} seconds")
            else:
                self.get_logger().warning("Start time is None, skipping mean time calculation.")
        elif self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 1:
            self.start_time = time.time()

    def _print_info(self,image):
        cv2.putText(image,f"{self.counter_seq}",(10,10),cv2.FONT_HERSHEY_DUPLEX,1.2,(255,0,0),1)
        return image



def main():
    rclpy.init()
    node = DataListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()