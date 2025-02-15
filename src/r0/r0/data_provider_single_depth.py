import rclpy
import rclpy.node
from sensor_msgs.msg import Image 


from cv_bridge import CvBridge
import cv2
import numpy as np

from glob import glob
import time

from .tools.param_load import load_depth_params,load_caminfo_params


class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("provider_depth")
        self.topic ,self.pub_periot ,self.pub_queue_size ,self.listen_periot ,self.frame_id ,self.data_path, self.typeruntime,self.typeshowtime  = load_depth_params()
        _,_,_,_,_,self.caminfo =load_caminfo_params()
        self.depth_names = []
        self._get_image_names()
        self.bridge = CvBridge()

        self.pub_depth = self.create_publisher(Image,self.topic,self.pub_queue_size)

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {self.pub_periot} second periot...")
        self.timer = self.create_timer(self.pub_periot,self.publish_callback)

            
    def sequence_read_callback(self):
        depth_frame = cv2.imread(self.depth_names[self.counter_seq],cv2.IMREAD_UNCHANGED)
        print(depth_frame.dtype)
        depth_frame = depth_frame.astype(np.float32)  # float32'ye dönüştür
        depth_frame = self._convert_depth_image_pix_to_mm(depth_frame)
        print(f"IMAGE DEPTH: {self.depth_names[self.counter_seq]} SHAPE: ",depth_frame.shape)

        self.counter_seq +=1
        return depth_frame

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        depth = self.sequence_read_callback()
        
        # depth
        msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding= self.typeruntime)  # encoding'i değiştir
        msg_depth.header.stamp = timestamp
        msg_depth.header.frame_id = self.frame_id

        self.pub_depth.publish(msg_depth)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")

    def _convert_depth_image_pix_to_mm(self,depth_image):

        [fx, _, cx,_, fy, cy,_, _, _] = self.caminfo.k
         
        factor = 5000 # for the 16-bit PNG files
        # OR: factor = 1 # for the 32-bit float images in the ROS bag files
        depth_image_mm = depth_image.copy()
        for v in range(self.caminfo.height):
            for u in range(self.caminfo.width):
                Z = depth_image[v,u] / factor
                #X = (u - cx) * Z / fx     pix:mm 1:1 so no need to convert x,y
                #Y = (v - cy) * Z / fy     pix:mm 1:1 so no need to convert x,y
                depth_image_mm[v,u] = Z 
        return depth_image_mm
    
    def _get_image_names(self):
        depth_names = sorted(glob(f"{self.data_path}/*"))
        self.depth_names = depth_names



def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

