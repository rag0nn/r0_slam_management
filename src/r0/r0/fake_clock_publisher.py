import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

class FakeClockPublisher(Node):
    def __init__(self):
        super().__init__('fake_clock_publisher')
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.01, self.publish_clock)  # 100 Hz yayınlama
        self.simulated_time = time.time()

    def publish_clock(self):
        # Simüle edilmiş zaman oluştur
        now = self.get_clock().now()
        clock_msg = Clock()
        clock_msg.clock = now.to_msg()  # Simüle edilmiş zaman
        self.publisher.publish(clock_msg)
        self.get_logger().info(f"Published fake time: {clock_msg.clock}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
