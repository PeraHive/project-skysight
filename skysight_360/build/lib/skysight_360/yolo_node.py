import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from ament_index_python.packages import get_package_share_directory
import os

class YoloDirectionNode(Node):
    def __init__(self):
        super().__init__('yolo_direction_node')

        # Load YOLO model (replace with your custom .pt if needed)
        package_share_dir = get_package_share_directory('skysight_360')
        model_path = os.path.join(package_share_dir, 'models', 'your_model.pt')
        self.model = YOLO(model_path)

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Subscriber to raw camera images
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publisher for processed images
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_yolo',
            10)

        # Publisher for dx, dy values
        self.offset_pub = self.create_publisher(
            Int32MultiArray,
            '/person_offset',
            10)

        self.get_logger().info("YOLO Direction Node started: listening on /camera/image_raw")

    def image_callback(self, msg):
        # Convert ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        frame_center = (w // 2, h // 2)

        dx, dy = 0, 0  # default (no detection)

        # Run YOLO inference
        results = self.model(frame, stream=True)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if cls == 0 and conf > 0.5:  # person only
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    dx = cx - frame_center[0]
                    dy = cy - frame_center[1]

                    # Draw annotations
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
                    cv2.arrowedLine(frame, (cx, cy), frame_center, (0, 255, 255), 2)
                    cv2.putText(frame, f"dx={dx}, dy={dy}",
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 255, 0), 2)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # Publish dx, dy values
        offset_msg = Int32MultiArray()
        offset_msg.data = [dx, dy]
        self.offset_pub.publish(offset_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDirectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
