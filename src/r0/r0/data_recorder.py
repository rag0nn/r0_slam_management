import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rosbag2_py
from rclpy.serialization import serialize_message
import os
from datetime import datetime
from .tools.param_load import load_linked_listen_params,load_types


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        self.topic_rgb,self.topic_depth,self.topic_caminfo,self.bag_save_path = load_linked_listen_params()
        _,self.rgb_typeshowtime,_,self.depth_typeshowtime = load_types()
        
        # Bag dizinini oluştur
        self.bag_root_dir = os.path.join(os.path.expanduser('~'), self.bag_save_path)
        os.makedirs(self.bag_root_dir, exist_ok=True)
        
        # Zaman damgalı klasör oluştur
        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.bag_dir = os.path.join(self.bag_root_dir, timestamp)
        os.makedirs(self.bag_dir, exist_ok=True)
        
        # Parametre tanımla
        self.declare_parameter('bag_filepath', 
                             os.path.join(self.bag_dir, 'recorded_data'))
        
        # Parametreler
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('queue_size', 10)
        
        # Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = Subscriber(self, Image, self.topic_rgb)
        self.depth_sub = Subscriber(self, Image, self.topic_depth)
        self.caminfo_sub = Subscriber(self, CameraInfo, self.topic_caminfo)
        
        # Senkronizasyon
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.caminfo_sub],
            queue_size=self.get_parameter('queue_size').value,
            slop=self.get_parameter('sync_slop').value
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Bag writer
        storage_options = rosbag2_py.StorageOptions(
            uri=self.get_parameter('bag_filepath').value,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(storage_options, converter_options)

        # Create topics before writing
        topic_types = [
            (self.topic_rgb, "sensor_msgs/msg/Image"),
            (self.topic_depth, "sensor_msgs/msg/Image"),
            (self.topic_caminfo, "sensor_msgs/msg/CameraInfo")
        ]

        for topic_name, topic_type in topic_types:
            topic_info = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr'
            )
            self.writer.create_topic(topic_info)
        
        self.get_logger().info('Data Recorder Node started')
        
    def sync_callback(self, rgb_msg, depth_msg, caminfo_msg):
        try:
            timestamp = self.get_clock().now().nanoseconds
            
            # Write messages
            self.writer.write(
                self.topic_rgb,
                serialize_message(rgb_msg),
                timestamp
            )
            self.writer.write(
                self.topic_depth,
                serialize_message(depth_msg),
                timestamp
            )
            self.writer.write(
                self.topic_caminfo,
                serialize_message(caminfo_msg),
                timestamp
            )
            
            # Görüntüleme (opsiyonel)
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, self.rgb_typeshowtime)
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, self.depth_typeshowtime)
            cv_depth = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            
            cv2.imshow("RGB", cv_rgb)
            cv2.imshow("Depth", cv_depth)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in sync callback: {str(e)}')
    
    def __del__(self):
        cv2.destroyAllWindows()
        if hasattr(self, 'writer'):
            self.writer.close()



def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()