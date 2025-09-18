#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2, time

class PreprocessNode(Node):
    def __init__(self):
        super().__init__('preprocess_node')
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('rate', 3.0)           # Hz
        self.declare_parameter('jpeg_quality', 30)    # 0–100

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.rate = self.get_parameter('rate').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # Pub/Sub
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pub_optimized = self.create_publisher(CompressedImage, '/camera/image_optimized', 10)


        self.last_pub_time = 0.0

    def cb(self, msg: Image):
        now = time.time()
        if now - self.last_pub_time < 1.0 / self.rate:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        resized = cv2.resize(cv_img, (self.width, self.height))

        # Publish raw resized
        msg_out = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
        msg_out.header = msg.header

        # Publish compressed
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, enc = cv2.imencode('.jpg', resized, encode_param)
        if success:
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            comp_msg.data = enc.tobytes()
            self.pub_optimized.publish(comp_msg)

        self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PreprocessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
