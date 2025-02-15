import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    
from std_msgs.msg import Header


from cv_bridge import CvBridge
import cv2
import numpy as np

from glob import glob
import time



NODE_NAME = ""
TOPIC_NAME_PUB_RGB = ""
TOPIC_NAME_PUB_DEPTH = ""
TOPIC_NAME_PUB_CAMERA_INFO = ""
IMAGES_PATH = ""
DEPTH_IMAGES_PATH ="" 
PUBLISH_PERIOT ="" 
PUBLISH_QUEUE_SIZE = ""
FRAME_ID = ""
CAM_WIDTH,CAM_HEIGHT = 0,0
CAM_D,CAM_D_MATRIX,CAM_K_MATRIX = "",[],[]





class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rgb_images= []
        self.depth_images = []
        print("Images importing")
        self._get_images()
        print("Images imported")
        print("Lengths: ",len(self.depth_images)," ",len(self.rgb_images))
        self.bridge = CvBridge()

        self.pub_rgb = self.create_publisher(Image,TOPIC_NAME_PUB_RGB,PUBLISH_QUEUE_SIZE)
        self.pub_depth = self.create_publisher(Image,TOPIC_NAME_PUB_DEPTH,PUBLISH_QUEUE_SIZE)
        self.pub_caminfo = self.create_publisher(CameraInfo,TOPIC_NAME_PUB_CAMERA_INFO,PUBLISH_QUEUE_SIZE)

        self.static_cam_info = self._get_camera_info()

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {PUBLISH_PERIOT} second periot...")
        self.timer = self.create_timer(PUBLISH_PERIOT,self.publish_callback)

            
    def sequence_read_callback(self):
        frame = self.rgb_images[self.counter_seq]
        depth_frame = self.depth_images[self.counter_seq]
        print(depth_frame.dtype)
        depth_frame = depth_frame.astype(np.float32)  # float32'ye dönüştür
        depth_frame = self._convert_depth_image_pix_to_mm(depth_frame)
        print(f"\nIMAGE RGB: {self.rgb_images[self.counter_seq]} SHAPE: ",frame.shape)
        print(f"IMAGE DEPTH: {self.depth_images[self.counter_seq]} SHAPE: ",depth_frame.shape)

        self.counter_seq +=1
        return frame,depth_frame

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        rgb,depth = self.sequence_read_callback()

        # rgb
        msg_rgb = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        msg_rgb.header.stamp = timestamp
        msg_rgb.header.frame_id = FRAME_ID
        #msg_rgb.header = Header(stamp=timestamp,frame_id=FRAME_ID)
        
        # depth
        msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding='32FC1')  # encoding'i değiştir
        msg_depth.header.stamp = timestamp
        msg_depth.header.frame_id = FRAME_ID
        #msg_depth.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # camera info
        self.static_cam_info.header.stamp = timestamp
        self.static_cam_info.header.frame_id = FRAME_ID
        #self.static_cam_info.header = Header(stamp=timestamp,frame_id=FRAME_ID)


        # publish
        self.pub_rgb.publish(msg_rgb)
        self.pub_depth.publish(msg_depth)
        self.pub_caminfo.publish(self.static_cam_info)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")

    def _get_camera_info(self):
        msg = CameraInfo()
        msg.width,msg.height = 640,480
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0263704224826013,
                                -0.10008645619921,
                                0.00313758409632316,
                                0.00242072236844001,
                                0.0]
        msg.k = [
            537.960321985614,0.0,319.183641033155,
            0.0,539.597659163239,247.053820358135,
            0.0,0.0,1.0
        ]
        msg.r = [
            1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,1.0
        ]
        msg.p = [
            535.43310546875,0.0,320.106652814575,
            0.0,0.0,539.212524414062,
            247.632132204719,0.0,0.0,
            0.0,1.0,0.0
        ]
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False
        return msg
    
    def _convert_depth_image_pix_to_mm(self,depth_image):
        CAM_K_MATRIX = [
            537.960321985614,0.0,319.183641033155,
            0.0,539.597659163239,247.053820358135,
            0.0,0.0,1.0
        ]
        [fx, _, cx,_, fy, cy,_, _, _] = CAM_K_MATRIX
         
        factor = 5000 # for the 16-bit PNG files
        # OR: factor = 1 # for the 32-bit float images in the ROS bag files
        depth_image_mm = depth_image.copy()
        for v in range(CAM_HEIGHT):
            for u in range(CAM_WIDTH):
                Z = depth_image[v,u] / factor
                #X = (u - cx) * Z / fx     pix:mm 1:1 so no need to convert x,y
                #Y = (v - cy) * Z / fy     pix:mm 1:1 so no need to convert x,y
                depth_image_mm[v,u] = Z 
        return depth_image_mm
    
    def _get_images(self):
        rgb_names = sorted(glob(f"{IMAGES_PATH}/*"))        
        depth_names = sorted(glob(f"{DEPTH_IMAGES_PATH}/*"))
        for i in rgb_names:
            self.rgb_images.append(cv2.imread(i))
        for i in depth_names:
            self.depth_images.append(cv2.imread(i,cv2.IMREAD_UNCHANGED))








def get_params():
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        import yaml

        global NODE_NAME, TOPIC_NAME_PUB_RGB, TOPIC_NAME_PUB_DEPTH, TOPIC_NAME_PUB_CAMERA_INFO
        global IMAGES_PATH, DEPTH_IMAGES_PATH, PUBLISH_PERIOT, PUBLISH_QUEUE_SIZE, FRAME_ID
        global CAM_D,CAM_D_MATRIX,CAM_K_MATRIX,CAM_WIDTH,CAM_HEIGHT

        package_name = 'r0'
        path = os.path.join(
            get_package_share_directory(package_name), 'config', 'const_params.yaml'
        )
        # YAML dosyasını oku
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        NODE_NAME = data["node_names"]["data_provider"]
        TOPIC_NAME_PUB_RGB = data["topics"]["image_rgb"]
        TOPIC_NAME_PUB_DEPTH = data["topics"]["image_depth"]
        TOPIC_NAME_PUB_CAMERA_INFO = data["topics"]["camera_info"]
        IMAGES_PATH = data["data_read"]["rgb_path"]
        DEPTH_IMAGES_PATH = data["data_read"]["depth_path"]
        PUBLISH_PERIOT = 0.003
        PUBLISH_QUEUE_SIZE = data["publish_config"]["publish_queue_size"]
        FRAME_ID = "openni_rgb_optical_frame"

        CAM_WIDTH =data["data_read"]["camera_info"]["resolution"]["width"]
        CAM_HEIGHT = data["data_read"]["camera_info"]["resolution"]["height"]
        CAM_D =data["data_read"]["camera_info"]["distortion_model"]
        CAM_D_MATRIX = data["data_read"]["camera_info"]["distortion_coefficients"]
        CAM_K_MATRIX = data["data_read"]["camera_info"]["intrinsic_matrix"]
    except:
        raise Exception("Params not loaded successfully!")


def main():
    get_params()
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

