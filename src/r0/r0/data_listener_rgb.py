import rclpy
import rclpy.node
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge
import time
from .tools.param_load import load_rgb_params

MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("lsitener_rgb")
        self.bridge = CvBridge()
        self.topic ,_,_,self.listen_periot ,_,_,self.typeruntime,self.typeshowtime= load_rgb_params()

        self.rgb_sub = self.create_subscription(
            Image,
            self.topic,
            self.listen_callback,
            self.listen_periot
        )

        self.counter_seq = 0
        self.get_logger().info(f"Image sequence listening with {self.listen_periot} listen periot...")


    def listen_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.typeshowtime)
            self.get_logger().info(f"\nDATA:{self.counter_seq} stamp: {msg.header.stamp}")
            
            cv_image = self._print_info(cv_image)

            cv2.imshow("RGB Image", cv_image)
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
        cv2.putText(image,f"{self.counter_seq}",(20,20),cv2.FONT_HERSHEY_DUPLEX,1,(255,0,0),1)
        return image


def main():
    rclpy.init()
    node = DataListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()