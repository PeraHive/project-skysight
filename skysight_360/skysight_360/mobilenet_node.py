#!/usr/bin/env python3
import os
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


# VOC class names used by MobileNet-SSD Caffe
CLASS_NAMES = {
    0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
    5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
    10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
    14: 'motorbike', 15: 'person', 16: 'pottedplant',
    17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'
}


class HumanDetectorLite(Node):
    """
    MobileNet-SSD (Caffe) person detector, optimized for Raspberry Pi.
    - Sub : /camera/image_optimized (CompressedImage)
    - Pub : /camera/image_yolo (Image, annotated)
    - Pub : /person_offset (Point) -> x=dx, y=dy, z=height_px
    - Pub : /person_height_norm (Float32) -> bbox_height / frame_height
    """

    def __init__(self):
        super().__init__('mobilenet_node')

        # === Parameters ===
        self.declare_parameter('conf_thresh', 0.5)          # match your sample
        self.declare_parameter('frame_skip', 1)             # process every frame initially
        self.declare_parameter('draw_all_classes', False)   # default off (person-only overlay)
        self.declare_parameter('person_only', True)         # skip non-person classes entirely
        self.declare_parameter('max_skip_no_person', 5)     # up to this skip when idle
        self.declare_parameter('no_person_boost_after', 15) # frames with no person before boosting
        self.declare_parameter('prototxt_path', '')         # optional absolute override
        self.declare_parameter('caffemodel_path', '')       # optional absolute override

        self.conf_thresh = float(self.get_parameter('conf_thresh').value)
        self.frame_skip = int(self.get_parameter('frame_skip').value)
        self.draw_all = bool(self.get_parameter('draw_all_classes').value)
        self.person_only = bool(self.get_parameter('person_only').value)
        self.max_skip_no_person = int(self.get_parameter('max_skip_no_person').value)
        self.no_person_boost_after = int(self.get_parameter('no_person_boost_after').value)

        proto_param = self.get_parameter('prototxt_path').get_parameter_value().string_value
        weights_param = self.get_parameter('caffemodel_path').get_parameter_value().string_value

        # === Resolve model paths ===
        if proto_param and weights_param and os.path.exists(proto_param) and os.path.exists(weights_param):
            proto = proto_param
            weights = weights_param
        else:
            share_dir = get_package_share_directory('skysight_360')
            models_dir = os.path.join(share_dir, 'models')
            proto = os.path.join(models_dir, 'MobileNetSSD_deploy.prototxt')
            weights = os.path.join(models_dir, 'MobileNetSSD_deploy.caffemodel')

        if not (os.path.exists(proto) and os.path.exists(weights)):
            self.get_logger().error(
                "Missing MobileNet-SSD model files in skysight_360/models/:\n"
                "  - MobileNetSSD_deploy.prototxt\n"
                "  - MobileNetSSD_deploy.caffemodel\n"
                "Or supply absolute paths via prototxt_path/caffemodel_path params."
            )
            raise FileNotFoundError("MobileNet-SSD files not found")

        self.get_logger().info(f"Using prototxt : {proto}")
        self.get_logger().info(f"Using caffemodel: {weights}")

        # === Load DNN ===
        self.net = cv2.dnn.readNetFromCaffe(proto, weights)
        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception:
            pass

        self.bridge = CvBridge()

        # === ROS I/O ===
        self.sub = self.create_subscription(
            CompressedImage, '/camera/image_optimized', self.image_cb, 10
        )
        self.pub_img = self.create_publisher(Image, '/camera/annotated', 10)
        self.pub_offset = self.create_publisher(Point, '/detected/offset', 10)
        # self.pub_height_norm = self.create_publisher(Float32, '/person_height_norm', 10)

        # === Buffer + worker ===
        self._lock = threading.Lock()
        self._latest = None
        self._running = True
        self._no_person_streak = 0

        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        # Warm-up forward (same blob settings as your example)
        self._warmup()

        self.get_logger().info("MobileNet-SSD detector started (person-only + adaptive throttling).")

    def _warmup(self):
        dummy = np.zeros((300, 300, 3), dtype=np.uint8)
        blob = cv2.dnn.blobFromImage(
            dummy, scalefactor=1/127.5, size=(300, 300),
            mean=(127.5, 127.5, 127.5), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        _ = self.net.forward()

    def image_cb(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR
            if frame is None:
                self.get_logger().warn("cv2.imdecode returned None")
                return
            with self._lock:
                self._latest = frame
        except Exception as e:
            self.get_logger().error(f"CompressedImage decode failed: {e}")

    def _loop(self):
        frame_idx = 0
        while rclpy.ok() and self._running:
            with self._lock:
                frame = None if self._latest is None else self._latest.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            frame_idx += 1
            if self.frame_skip > 1 and (frame_idx % self.frame_skip != 0):
                continue

            h, w = frame.shape[:2]

            # === EXACT OpenCV example blob settings ===
            blob = cv2.dnn.blobFromImage(
                frame,
                scalefactor=1/127.5,
                size=(300, 300),
                mean=(127.5, 127.5, 127.5),
                swapRB=True,   # matches your sample code
                crop=False
            )
            self.net.setInput(blob)
            detections = self.net.forward()  # [1,1,N,7]

            frame_center = (w // 2, h // 2)
            best_person = None  # (x1,y1,x2,y2,conf)

            # --- post-processing (person-only path saves CPU) ---
            for i in range(detections.shape[2]):
                conf = float(detections[0, 0, i, 2])
                if conf < self.conf_thresh:
                    continue

                class_id = int(detections[0, 0, i, 1])

                if self.person_only and class_id != 15:
                    # ignore non-person entirely
                    continue

                x1 = int(detections[0, 0, i, 3] * w)
                y1 = int(detections[0, 0, i, 4] * h)
                x2 = int(detections[0, 0, i, 5] * w)
                y2 = int(detections[0, 0, i, 6] * h)

                x1 = max(0, min(x1, w - 1)); y1 = max(0, min(y1, h - 1))
                x2 = max(0, min(x2, w - 1)); y2 = max(0, min(y2, h - 1))

                # Draw all classes (optional; can cost a bit of CPU)
                if not self.person_only and self.draw_all and class_id in CLASS_NAMES:
                    label = f"{CLASS_NAMES[class_id]}: {conf:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))
                    (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    y_text = max(y1, th)
                    cv2.rectangle(frame, (x1, y_text - th), (x1 + tw, y_text + baseline), (0, 0, 0), cv2.FILLED)
                    cv2.putText(frame, label, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

                if class_id == 15:  # 'person'
                    if (best_person is None) or (conf > best_person[4]):
                        best_person = (x1, y1, x2, y2, conf)

            # Defaults
            dx = dy = 0.0
            height_px = 0.0
            height_norm = 0.0

            if best_person is not None:
                x1, y1, x2, y2, conf = best_person
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                dx = float(cx - frame_center[0])
                dy = float(cy - frame_center[1])
                height_px = float(y2 - y1)
                height_norm = (y2 - y1) / float(h) if h > 0 else 0.0

                # Minimal helpful overlay for person
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.arrowedLine(frame, (cx, cy), frame_center, (0, 255, 255), 2, tipLength=0.2)
                cv2.putText(frame, f"person {conf:.2f} dx={int(dx)} dy={int(dy)} h={int(height_px)}",
                            (max(0, x1), max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                # Reset idle streak -> be responsive
                self._no_person_streak = 0
            else:
                # No person, ramp toward low-power mode
                self._no_person_streak += 1

            # === Adaptive throttling to save power ===
            if self._no_person_streak >= self.no_person_boost_after:
                self.frame_skip = min(self.frame_skip + 1, self.max_skip_no_person)
            else:
                self.frame_skip = max(1, self.frame_skip - 1)

            # Publish annotated frame
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_img.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")

            # Publish person offset/height
            self.pub_offset.publish(Point(x=dx, y=dy, z=height_norm))
            # self.pub_height_norm.publish(Float32(data=height_norm))

            # Be nice to executor
            time.sleep(0.003)

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_worker') and self._worker.is_alive():
            self._worker.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectorLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
