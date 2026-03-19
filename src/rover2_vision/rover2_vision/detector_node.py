#!/usr/bin/env python3
"""
Object detection node using YOLOv8n.
Subscribes to /image_raw, runs inference, publishes detections as JSON.
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('model', 'yolov8n')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('debug_image', True)

        model_name = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class').value
        # YOLO often confuses similar objects — map aliases
        self._class_aliases = {
            'cup': {'cup', 'vase'},
            'bottle': {'bottle', 'vase'},
        }
        self.accepted_classes = self._class_aliases.get(self.target_class, {self.target_class})
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_image = self.get_parameter('debug_image').value

        # Load YOLO model
        self.get_logger().info(f'Loading {model_name} model...')
        from ultralytics import YOLO
        self.model = YOLO(f'{model_name}.pt')
        self.get_logger().info(f'Model loaded. Target class: {self.target_class}')

        # ROS interfaces
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.det_pub = self.create_publisher(String, '/detections', 10)
        if self.debug_image:
            self.debug_pub = self.create_publisher(Image, '/detector/debug_image', 1)

        self.last_inference_time = 0.0
        self.min_interval = 1.0 / self.publish_rate

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_inference_time < self.min_interval:
            return
        self.last_inference_time = now

        # Convert ROS image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run inference
        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            verbose=False,
            imgsz=320,  # 320 = ~5 FPS on Pi 5 (480 = ~2.7 FPS, too slow)
        )

        # Parse results
        detections = []
        for r in results:
            for box in r.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                if class_name not in self.accepted_classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                bbox_w = x2 - x1
                bbox_h = y2 - y1
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                detections.append({
                    'class_name': class_name,
                    'confidence': round(confidence, 3),
                    'bbox_x': round(x1),
                    'bbox_y': round(y1),
                    'bbox_w': round(bbox_w),
                    'bbox_h': round(bbox_h),
                    'bbox_center_x': round(center_x),
                    'bbox_center_y': round(center_y),
                })

        # Pick the largest detection (closest object)
        if len(detections) > 1:
            detections.sort(key=lambda d: d['bbox_w'] * d['bbox_h'], reverse=True)
            detections = [detections[0]]

        # Publish detections
        det_msg = String()
        det_msg.data = json.dumps({
            'timestamp': now,
            'target_class': self.target_class,
            'detections': detections,
            'image_width': frame.shape[1],
            'image_height': frame.shape[0],
        })
        self.det_pub.publish(det_msg)

        # Publish debug image
        if self.debug_image and detections:
            for d in detections:
                x1 = d['bbox_x']
                y1 = d['bbox_y']
                x2 = x1 + d['bbox_w']
                y2 = y1 + d['bbox_h']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{d['class_name']} {d['confidence']:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass

        if detections:
            d = detections[0]
            self.get_logger().info(
                f'{d["class_name"]} conf={d["confidence"]:.2f} '
                f'center=({d["bbox_center_x"]},{d["bbox_center_y"]}) '
                f'h={d["bbox_h"]}px',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
