import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class YOLOInfoSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_info_subscriber')
        self.subscription = self.create_subscription(
            String,
            'yolo/detected_info',
            self.info_callback,
            10)
        self.subscription  # 방지: linter 경고(구독 객체 유지)

    def info_callback(self, msg):
        self.get_logger().info(f'Received YOLO detected info: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOInfoSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
