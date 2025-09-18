#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import cv2
import os
import threading
import time
import numpy as np
from cv_bridge import CvBridge   # still needed for publishing Image

class YoloDirectionNode(Node):
    def __init__(self):
        super().__init__('yolo_direction_node')

        # Load YOLO model
        package_share_dir = get_package_share_directory('skysight_360')
        model_path = os.path.join(package_share_dir, 'models', 'yolov8n.pt')
        self.model = YOLO(model_path)

        # CvBridge only for publishing raw Image
        self.bridge = CvBridge()

        # Subscribers / Publishers
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_optimized',   # now expects CompressedImage
            self.image_callback,
            10)
        self.image_pub = self.create_publisher(Image, '/camera/image_yolo', 10)
        self.offset_pub = self.create_publisher(Int32MultiArray, '/person_offset', 10)

        # Shared frame buffer
        self.frame_lock = threading.Lock()
        self.latest_frame = None

        # Optimization params
        self.frame_skip = 3         # process 1 in every 3 frames
        self.resize_width = 640     # resize before YOLO
        self.conf_thresh = 0.5      # confidence threshold
        self.running = True

        # Start worker thread
        self.worker = threading.Thread(target=self.yolo_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("YOLO Direction Node started (CompressedImage input)")

    def image_callback(self, msg: CompressedImage):
        """Decode compressed image and store latest frame."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
            else:
                self.get_logger().warn("cv2.imdecode returned None")
        except Exception as e:
            self.get_logger().error(f"CompressedImage decode failed: {e}")

    def yolo_loop(self):
        """Background thread: run YOLO inference at steady rate."""
        frame_count = 0
        while rclpy.ok() and self.running:
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1
            if frame_count % self.frame_skip != 0:
                continue  # skip frames for speed

            h, w, _ = frame.shape
            frame_center = (w // 2, h // 2)
            dx, dy = 0, 0

            # Resize before inference
            scale = self.resize_width / w
            resized = cv2.resize(frame, (self.resize_width, int(h * scale)))

            # Run YOLO
            try:
                results = self.model(resized, stream=True, verbose=False)
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}")
                continue

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    if cls == 0 and conf > self.conf_thresh:  # person
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = int((x1 + x2) / 2 / scale)
                        cy = int((y1 + y2) / 2 / scale)
                        dx = cx - frame_center[0]
                        dy = cy - frame_center[1]

                        # Draw (optional)
                        cv2.rectangle(frame, (int(x1/scale), int(y1/scale)),
                                      (int(x2/scale), int(y2/scale)), (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.arrowedLine(frame, (cx, cy), frame_center, (0, 255, 255), 2)
                        cv2.putText(frame, f"dx={dx}, dy={dy}",
                                    (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (255, 255, 0), 2)

            # Publish annotated image (raw Image for RViz/Foxglove)
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")

            # Publish offsets
            offset_msg = Int32MultiArray()
            offset_msg.data = [dx, dy]
            self.offset_pub.publish(offset_msg)

            time.sleep(0.01)  # ~100 Hz max loop

    def destroy_node(self):
        self.running = False
        if self.worker.is_alive():
            self.worker.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
