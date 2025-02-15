from .depth_estimation.run import MidasEstimator

import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    


from cv_bridge import CvBridge
import cv2
import numpy as np

from glob import glob
import time

from .tools.param_load import load_rgbd_linked_params


class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__("provider_rgbd_vie_estimation")

        rgb_params,depth_params,caminfo_params = load_rgbd_linked_params()
        self.rgb_topic ,self.rgb_pub_periot ,self.rgb_pub_queue_size ,_ ,self.rgb_frame_id,self.rgb_data_path, self.rgb_typeruntime,self.rgb_typeshowtime = rgb_params
        self.depth_topic ,self.depth_pub_periot ,self.depth_pub_queue_size ,_ ,self.depth_frame_id ,_,self.depth_typeruntime,self.depth_typeshowtime = depth_params
        self.caminfo_topic,self.caminfo_pub_periot,self.caminfo_pub_queue_size,_,self.caminfo_frame_id,self.caminfo_camerainfo = caminfo_params
        
        self.static_cam_info = self.caminfo_camerainfo

        self.rgb_names = []
        self._get_image_names()

        self.depth_estimator = MidasEstimator("/home/rag0n/Desktop/r0_workspace/src/r0/r0/depth_estimation/midas_v21_small_256.pt")
        self.bridge = CvBridge()
        self.pub_rgb = self.create_publisher(Image,self.rgb_topic,self.rgb_pub_queue_size)
        self.pub_depth = self.create_publisher(Image,self.depth_topic,self.depth_pub_queue_size)
        self.pub_caminfo = self.create_publisher(CameraInfo,self.caminfo_topic,self.caminfo_pub_queue_size)

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {self.depth_pub_periot} second periot...")
        self.timer = self.create_timer(self.depth_pub_periot,self.publish_callback)

            
    def sequence_read_callback(self):
        frame = cv2.imread(self.rgb_names[self.counter_seq])
        depth_frame = self.depth_estimator.estimate(frame)
        depth_frame = 255 - depth_frame
        depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_RGB2GRAY).astype(np.float32)   
        depth_frame = self._convert_depth_image_pix_to_mm(depth_frame)
        print(f"\nIMAGE RGB: {self.rgb_names[self.counter_seq]} SHAPE: ",frame.shape)
        print(f"IMAGE DEPTH: {self.rgb_names[self.counter_seq]} SHAPE: ",depth_frame.shape)
        self.counter_seq +=1
        return frame,depth_frame

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        rgb,depth = self.sequence_read_callback()

        # rgb
        msg_rgb = self.bridge.cv2_to_imgmsg(rgb, encoding=self.rgb_typeruntime)
        msg_rgb.header.stamp = timestamp
        msg_rgb.header.frame_id = self.rgb_frame_id
        
        # depth
        msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding=self.depth_typeruntime)
        msg_depth.header.stamp = timestamp
        msg_depth.header.frame_id = self.depth_frame_id

        # camera info
        self.static_cam_info.header.stamp = timestamp
        self.static_cam_info.header.frame_id = self.caminfo_frame_id

        # publish
        self.pub_rgb.publish(msg_rgb)
        self.pub_depth.publish(msg_depth)
        self.pub_caminfo.publish(self.static_cam_info)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")
    
    def _convert_depth_image_pix_to_mm(self,depth_image):
        [fx, _, cx,_, fy, cy,_, _, _] = self.caminfo_camerainfo.k
         
        factor = 5000 # for the 16-bit PNG files
        # OR: factor = 1 # for the 32-bit float images in the ROS bag files
        depth_image_mm = depth_image.copy()
        for v in range(self.caminfo_camerainfo.height):
            for u in range(self.caminfo_camerainfo.width):
                Z = depth_image[v,u] / factor
                #X = (u - cx) * Z / fx     pix:mm 1:1 so no need to convert x,y
                #Y = (v - cy) * Z / fy     pix:mm 1:1 so no need to convert x,y
                depth_image_mm[v,u] = Z 
        return depth_image_mm
    
    def _get_image_names(self):
        rgb_names = sorted(glob(f"{self.rgb_data_path}/*"))        
        self.rgb_names = rgb_names


def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

