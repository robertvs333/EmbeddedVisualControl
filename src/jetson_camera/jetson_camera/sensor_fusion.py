#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import String, Float32

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

import mediapipe as mp
from ultralytics import YOLO

class HUDIntegratedDetectionNode(Node):
    def __init__(self):
        super().__init__('hud_integrated_detection_node')
        self.bridge = CvBridge()

        # 1. Initialize Vision Engines
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detector = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)
        self.object_model = YOLO('yolov8n.pt')
        
        # 2. State Control Variables
        self.is_scanning = False
        self.sample_size = 10  
        self.state_history = []
        self.distance_history = []
        self.camera_fov_rad = 1.05  
        self.min_face_confidence = 0.80
        self.min_object_confidence = 0.65
        self.centering_time_limit = 5.0  # seconds before giving up on a candidate
        self.centering_start_time = None

        # 3. ROS 2 Communication
        self.trigger_sub = self.create_subscription(String, '/detection/trigger', self.trigger_callback, 10)
        self.result_pub = self.create_publisher(String, '/detection/final_result', 10)
        self.angle_pub = self.create_publisher(Float32, '/detection/tracking_angle', 10)

        self.image_sub = message_filters.Subscriber(self, CompressedImage, '/camera/image_undistorted', qos_profile=qos_profile_sensor_data)
        self.tof_sub = message_filters.Subscriber(self, Range, '/tof/distance', qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.tof_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)

        self.tof_threshold_m = 2.0  
        
        # Create a persistent OpenCV window
        cv2.namedWindow("Sensor Fusion HUD Monitor", cv2.WINDOW_NORMAL)
        self.get_logger().info("HUD Diagnostic Monitor Spawned. Standby...")

    def trigger_callback(self, msg):
        if self.is_scanning:
            return
        self.get_logger().info("Trigger received! Initiating HUD tracking graphics...")
        self.state_history.clear()
        self.distance_history.clear()
        self.is_scanning = True

    def is_spatial_match(self, bbox, frame_width, frame_height):
        x, y, w, h = bbox
        return (x <= frame_width // 2 <= x + w) and (y <= frame_height // 2 <= y + h)

    def send_tracking_angle(self, bbox_x, bbox_w, frame_w):
        bbox_center_x = bbox_x + (bbox_w / 2)
        normalized_error = 0.5 - (bbox_center_x / frame_w)
        target_angle = normalized_error * self.camera_fov_rad
        
        angle_msg = Float32()
        angle_msg.data = float(target_angle)
        self.angle_pub.publish(angle_msg)

    def stop_robot(self):
        angle_msg = Float32()
        angle_msg.data = 0.0
        self.angle_pub.publish(angle_msg)

    def synchronized_callback(self, image_msg, tof_msg):
        try:
            # Decode the compressed ROS image into an OpenCV BGR image
            cv_img = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            ih, iw, _ = cv_img.shape
            tof_distance = tof_msg.range
            
            # Draw static crosshairs for the ToF center-beam alignment
            cx, cy = iw // 2, ih // 2
            cv2.line(cv_img, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 2)
            cv2.line(cv_img, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 2)

            # --- PASSIVE MODE: Just stream camera data if not triggered ---
            if not self.is_scanning:
                cv2.putText(cv_img, "SYSTEM STATE: STANDBY", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(cv_img, f"ToF Dist: {tof_distance:.2f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow("Sensor Fusion HUD Monitor", cv_img)
                cv2.waitKey(1)
                return

            # --- ACTIVE MODE: Trigger is running ---
            frame_type = "none"
            frame_confidence = 0.0
            centered = False
            target_bbox = None

            # 1. MediaPipe Face Detection
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            face_results = self.face_detector.process(rgb_img)

            if face_results.detections:
                best_face = max(face_results.detections, key=lambda d: d.score[0])
                face_confidence = best_face.score[0]
                if face_confidence >= self.min_face_confidence:
                    bboxC = best_face.location_data.relative_bounding_box
                    fx, fy = max(0, int(bboxC.xmin * iw)), max(0, int(bboxC.ymin * ih))
                    fw, fh = int(bboxC.width * iw), int(bboxC.height * ih)
                    frame_type = "face"
                    frame_confidence = face_confidence
                    target_bbox = (fx, fy, fw, fh)
                    centered = self.is_spatial_match(target_bbox, iw, ih)
                else:
                    frame_type = "none"

            # 2. YOLO Object Fallback
            if frame_type == "none":
                yolo_results = self.object_model(cv_img, verbose=False)
                if len(yolo_results[0].boxes) > 0:
                    best_box = max(yolo_results[0].boxes, key=lambda b: b.conf[0].item())
                    object_confidence = best_box.conf[0].item()
                    if object_confidence >= self.min_object_confidence:
                        x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
                        frame_type = "object"
                        frame_confidence = object_confidence
                        target_bbox = (x1, y1, x2 - x1, y2 - y1)
                        centered = self.is_spatial_match(target_bbox, iw, ih)
                    else:
                        frame_type = "none"

            # 3. HUD Graphics & State Pipeline Integration
            if frame_type != "none" and target_bbox:
                x, y, w, h = target_bbox
                # Yellow if moving/off-center, Solid Green if locked onto the crosshair
                box_color = (0, 255, 0) if centered else (0, 255, 255)
                cv2.rectangle(cv_img, (x, y), (x + w, y + h), box_color, 2)
                cv2.putText(cv_img, f"{frame_type.upper()} ({frame_confidence:.2f})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            if frame_type != "none" and not centered:
                if self.centering_start_time is None:
                    self.centering_start_time = time.time()
                    self.state_history.clear()
                    self.distance_history.clear()

                elapsed = time.time() - self.centering_start_time
                self.send_tracking_angle(target_bbox[0], target_bbox[2], iw)

                if elapsed >= self.centering_time_limit:
                    self.get_logger().info(
                        f"Centering timeout after {elapsed:.1f}s. Abandoning target and stopping scan."
                    )
                    self.stop_robot()
                    self.is_scanning = False
                    self.centering_start_time = None
                    self.state_history.clear()
                    self.distance_history.clear()
                    return

                # Update Status Text
                cv2.putText(cv_img, "SYSTEM STATE: ALIGNING CHASSIS", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                self.centering_start_time = None
                # Target is centered or no target exists - record snapshot frames
                if centered:
                    self.stop_robot()
                    tof_presence = tof_distance < self.tof_threshold_m
                    boost = 0.15 if tof_presence else (-0.30 if frame_type == "face" else -0.10)
                    confidence = np.clip(frame_confidence + boost, 0.0, 1.0)
                    
                    self.state_history.append((frame_type, confidence))
                    self.distance_history.append(tof_distance)
                    
                    cv2.putText(cv_img, "SYSTEM STATE: CAPTURING SNAPSHOT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    self.state_history.append(("none", 1.0))
                    self.distance_history.append(0.0)
                    cv2.putText(cv_img, "SYSTEM STATE: SCANNING (EMPTY)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # Add a visual frame buffering progress bar to the screen
                progress = len(self.state_history)
                cv2.rectangle(cv_img, (20, 90), (220, 105), (50, 50, 50), -1)
                cv2.rectangle(cv_img, (20, 90), (20 + (progress * 20), 105), (0, 255, 0), -1)
                cv2.putText(cv_img, f"Samples: {progress}/{self.sample_size}", (230, 103), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Draw Live ToF Readout 
            cv2.putText(cv_img, f"Live ToF Distance: {tof_distance:.2f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # Render frame to window
            cv2.imshow("Sensor Fusion HUD Monitor", cv_img)
            cv2.waitKey(1) # Refresh window context

            if len(self.state_history) >= self.sample_size:
                self.evaluate_and_publish_final_result()

        except Exception as e:
            self.get_logger().error(f"HUD Monitor Core Failure: {e}")
            self.is_scanning = False

    def evaluate_and_publish_final_result(self):
        self.is_scanning = False
        self.stop_robot()

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
        self.get_logger().info(f"Payload Published: {output_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = HUDIntegratedDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()