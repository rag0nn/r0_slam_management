import rclpy
import rclpy.node
from sensor_msgs.msg import Image
import time
import cv2
from cv_bridge import CvBridge

NODE_NAME = "stereo_example_rgbd_subscriber"
TOPIC_NAME = "stereo_camera/rgbd_image"
LISTEN_PERIOT = 10
MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self.sub_node = self.create_subscription(
            Image,
            TOPIC_NAME,
            self.listen_callback,
            LISTEN_PERIOT
        )

        self.counter_seq = 0

        self.get_logger().info(f"Image listening {TOPIC_NAME} topic with {LISTEN_PERIOT} listen periot...")


    def listen_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"\nDATA:{self.counter_seq} stamp: {msg.header.stamp}")

            print("SHAPE: ", cv_image.shape)
            print("TYPE: ", cv_image.dtype)
            print(cv_image)
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





def main():
    rclpy.init()
    node = DataListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()