import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DDSImageListener(Node):
    def __init__(self):
        super().__init__('dds_image_publisher')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)

        # Subscribe to DDS-like topic (ROS 2 topic simulating DDS input)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info('DDS Image Listener Node has started.')

    def listener_callback(self, msg):
        try:
            # Directly decode JPEG data from the ROS 2 Image message
            jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode JPEG image.')
                return

            # Process image (e.g., display or modify)
            cv2.imshow("DDS Image Viewer", cv_image)
            cv2.waitKey(1)

            # Optionally republish the decoded image
            decoded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_.publish(decoded_msg)
            # self.get_logger().info(f'Image received and republished: {cv_image.shape[1]}x{cv_image.shape[0]}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DDSImageListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DDS Image Listener Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
