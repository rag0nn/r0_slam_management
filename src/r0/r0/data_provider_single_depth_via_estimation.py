from .depth_estimation.run import MidasEstimator

import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    


from cv_bridge import CvBridge
import cv2
import numpy as np

import time

from .tools.param_load import load_rgbd_linked_params


class DataProvider(rclpy.node.Node):
    """
    Listens rgb image and publish depth frame with same timestamp
    """
    
    def __init__(self):
        super().__init__("provider_depth_via_estimation")
        rgb_params,depth_params,caminfo_params = load_rgbd_linked_params()
        self.rgb_topic ,_ ,_ ,self.listen_periot,_,_,self.rgb_typeruntime,self.rgb_typeshowtime = rgb_params
        self.depth_topic ,self.depth_pub_periot ,self.depth_pub_queue_size ,_ ,self.depth_frame_id ,_,self.depth_typeruntime,self.depth_typeshowtime = depth_params
        _,_,_,_,_,self.caminfo_camerainfo = caminfo_params
        
        self.depth_estimator = MidasEstimator("/home/rag0n/Desktop/r0_workspace/src/r0/r0/depth_estimation/midas_v21_small_256.pt")
        
        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(Image,self.rgb_topic,self.pipe_callback,self.depth_pub_queue_size)
        self.pub_depth = self.create_publisher(Image,self.depth_topic,self.depth_pub_queue_size)


        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {self.depth_pub_periot} second periot...")


            
    def process_callback(self,frame):
        depth_frame = self.depth_estimator.estimate(frame)
        depth_frame = 255 - depth_frame
        depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_RGB2GRAY).astype(np.float32)   
        depth_frame = self._convert_depth_image_pix_to_mm(depth_frame)
        print(f"DEPTH SHAPE: ",depth_frame.shape)
        self.counter_seq +=1
        return depth_frame

    def pipe_callback(self,msg):
        try:
            self.get_logger().info(f"Image got: {msg.header.stamp}")
            start_callback_time = time.time()

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding= self.rgb_typeruntime)
            depth = self.process_callback(cv_image)

            # depth
            msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding=self.depth_typeruntime)  # encoding'i değiştir
            msg_depth.header.stamp = msg.header.stamp
            msg_depth.header.frame_id = self.depth_frame_id

            # publish
            self.pub_depth.publish(msg_depth)

            end_callback_time = time.time()
            callback_duration = end_callback_time - start_callback_time
            self.get_logger().info(f"Total read and publish time: {callback_duration}")

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
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
    


def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
