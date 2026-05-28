#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String, Float32

import cv2
from cv_bridge import CvBridge
import numpy as np

import mediapipe as mp
from ultralytics import YOLO

class IntegratedDetectionQueryNode(Node):
    def __init__(self):
        super().__init__('integrated_detection_query_node')
        self.bridge = CvBridge()

        # 1. Initialize Vision Engines
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detector = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)
        self.object_model = YOLO('yolov8n.pt')
        
        # 2. State Control Variables
        self.is_scanning = False
        self.sample_size = 10  # Number of stable centered frames required before finalizing
        self.state_history = []
        self.distance_history = []
        
        self.camera_fov_rad = 1.05  # Roughly 60 degrees horizontal FOV

        # 3. ROS 2 Communication Setup
        self.trigger_sub = self.create_subscription(String, '/detection/trigger', self.trigger_callback, 10)
        
        self.result_pub = self.create_publisher(String, '/detection/final_result', 10)
        self.angle_pub = self.create_publisher(Float32, '/detection/tracking_angle', 10)

        # Sync listeners for continuous camera/ToF data stream
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_undistorted', qos_profile=qos_profile_sensor_data)
        self.tof_sub = message_filters.Subscriber(self, Range, '/tof/distance', qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.tof_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)

        self.tof_threshold_m = 2.0  
        self.get_logger().info("Integrated Fusion & Centering Node Active. Awaiting Trigger...")

    def trigger_callback(self, msg):
        """Wakes up the node execution loop."""
        if self.is_scanning:
            self.get_logger().warn("Scan already in progress.")
            return
            
        self.get_logger().info("Trigger received! Starting active alignment loop...")
        self.state_history.clear()
        self.distance_history.clear()
        self.is_scanning = True

    def is_spatial_match(self, bbox, frame_width, frame_height):
        """Checks if the target center intersects with the camera's center axis."""
        x, y, w, h = bbox
        center_x = frame_width // 2
        center_y = frame_height // 2
        return (x <= center_x <= x + w) and (y <= center_y <= y + h)

    def send_tracking_angle(self, bbox_x, bbox_w, frame_w):
        """Calculates displacement error and sends it to the Jetson wheel controller."""
        bbox_center_x = bbox_x + (bbox_w / 2)
        normalized_error = 0.5 - (bbox_center_x / frame_w)
        target_angle = normalized_error * self.camera_fov_rad
        
        angle_msg = Float32()
        angle_msg.data = float(target_angle)
        self.angle_pub.publish(angle_msg)

    def stop_robot(self):
        """Forces the Jetson's tracking node to brake immediately."""
        angle_msg = Float32()
        angle_msg.data = 0.0
        self.angle_pub.publish(angle_msg)

    def synchronized_callback(self, image_msg, tof_msg):
        # Guardrail: Avoid wasting laptop processing power when idle
        if not self.is_scanning:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            ih, iw, _ = cv_img.shape
            tof_distance = tof_msg.range
            
            frame_type = "none"
            frame_confidence = 0.0
            centered = False
            target_bbox = None

            # --- 1. Run MediaPipe Face Detection ---
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            face_results = self.face_detector.process(rgb_img)

            if face_results.detections:
                best_face = max(face_results.detections, key=lambda d: d.score[0])
                bboxC = best_face.location_data.relative_bounding_box
                fx = max(0, int(bboxC.xmin * iw))
                fy = max(0, int(bboxC.ymin * ih))
                fw = int(bboxC.width * iw)
                fh = int(bboxC.height * ih)
                
                frame_type = "face"
                frame_confidence = best_face.score[0]
                target_bbox = (fx, fy, fw, fh)
                centered = self.is_spatial_match(target_bbox, iw, ih)

            # --- 2. Fallback to YOLO Generic Objects ---
            if frame_type == "none":
                yolo_results = self.object_model(cv_img, verbose=False)
                if len(yolo_results[0].boxes) > 0:
                    best_box = max(yolo_results[0].boxes, key=lambda b: b.conf[0].item())
                    x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
                    
                    frame_type = "object"
                    frame_confidence = best_box.conf[0].item()
                    target_bbox = (x1, y1, x2 - x1, y2 - y1)
                    centered = self.is_spatial_match(target_bbox, iw, ih)

            # --- 3. Spatial Centering Gating Control Loop ---
            if frame_type != "none" and not centered:
                # Target is found but off-center: clear old snapshot queues and command a turn
                self.state_history.clear()
                self.distance_history.clear()
                
                self.send_tracking_angle(target_bbox[0], target_bbox[2], iw)
                self.get_logger().info(f"Target ({frame_type}) is off-center. Directing car adjustments...", throttle_duration_sec=0.5)
                return  # Exit early to wait for the next frame during alignment

            # --- 4. Snapshot Processing (Only active when centered or empty) ---
            if centered:
                self.stop_robot() # Target aligned! Lock the chassis down.
                
                tof_presence = tof_distance < self.tof_threshold_m
                if frame_type == "face":
                    boost = 0.15 if tof_presence else -0.30
                    confidence = np.clip(frame_confidence + boost, 0.0, 1.0)
                else:  # object
                    boost = 0.15 if tof_presence else -0.10
                    confidence = np.clip(frame_confidence + boost, 0.0, 1.0)
                    
                self.state_history.append((frame_type, confidence))
                self.distance_history.append(tof_distance)
            else:
                # No detections anywhere in the image plane
                self.state_history.append(("none", 1.0))
                self.distance_history.append(0.0)

            # --- 5. Evaluate Snapshot Windows ---
            if len(self.state_history) >= self.sample_size:
                self.evaluate_and_publish_final_result()

        except Exception as e:
            self.get_logger().error(f"Core Fusion Engine Failure: {e}")
            self.is_scanning = False

    def evaluate_and_publish_final_result(self):
        """Processes the clean data window, publishes one JSON block, and sleeps."""
        self.is_scanning = False
        self.stop_robot() # Redundant safety stop

        states = [item[0] for item in self.state_history]
        fused_state = max(set(states), key=states.count)

        matching_confidences = [item[1] for item in self.state_history if item[0] == fused_state]
        fused_confidence = np.mean(matching_confidences) if matching_confidences else 0.0

        if fused_state != "none":
            valid_distances = [d for d in self.distance_history if d > 0.0]
            fused_distance = np.mean(valid_distances) if valid_distances else 0.0
        else:
            fused_distance = 0.0

        output_msg = String()
        output_msg.data = f"{{\'_type\': \'{fused_state}\', \'confidence\': {fused_confidence:.2f}, \'distance_m\': {fused_distance:.2f}}}"
        self.result_pub.publish(output_msg)
        
        self.get_logger().info(f"=== SINGLE PAYLOAD BROADCAST COMPLETE ===")
        self.get_logger().info(f"Output: {output_msg.data}")
        self.get_logger().info("Shutting down vision engines. Entering low-power standby mode.\n")

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedDetectionQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()