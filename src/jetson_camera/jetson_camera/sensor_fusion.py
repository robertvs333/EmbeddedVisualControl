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

# MobileSAM imports (pip install mobile-sam)
from mobile_sam import sam_model_registry, SamAutomaticMaskGenerator

class HUDIntegratedDetectionNode(Node):
    def __init__(self):
        super().__init__('hud_integrated_detection_node')
        self.bridge = CvBridge()

        # 1. Initialize Vision Engines
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detector = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)

        # Initialize MobileSAM
        self.get_logger().info("Loading MobileSAM model...")
        sam_checkpoint = "mobile_sam.pt"
        model_type = "vit_t"
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.eval()

        # Dynamic SAM Polling variables
        self.sam_active_skip = 4      
        self.sam_idle_skip = 30       
        self.current_sam_skip = self.sam_active_skip
        
        self.sam_frame_counter = 0
        self.sam_last_bbox = None
        self.sam_last_confidence = 0.0
        self.sam_max_width = 640
        
        # Tracker Lifecycle Management
        self.tracker = None
        self.tracker_frame_counter = 0
        self.tracker_update_interval = 6  

        self.mask_generator = SamAutomaticMaskGenerator(
            sam,
            points_per_side=6,
            pred_iou_thresh=0.65,
            stability_score_thresh=0.75,
            min_mask_region_area=350,
        )
        self.get_logger().info("MobileSAM ready.")

        # 2. State Control Variables
        self.is_scanning = False
        self.sample_size = 10
        self.state_history = []
        self.distance_history = []
        self.camera_fov_rad = 1.05
        self.min_face_confidence = 0.80
        self.max_object_area_ratio = 0.5
        self.centering_time_limit = 5.0
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

    def create_tracker(self):
        if hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
            return cv2.legacy.TrackerCSRT_create()
        if hasattr(cv2, 'TrackerKCF_create'):
            return cv2.TrackerKCF_create()
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            return cv2.legacy.TrackerKCF_create()
        raise RuntimeError('No supported OpenCV tracker found')

    def init_tracker(self, cv_img, bbox):
        try:
            self.tracker = self.create_tracker()
            self.tracker.init(cv_img, tuple(bbox))
            self.tracker_frame_counter = 0  
        except Exception as e:
            self.get_logger().warning(f"Tracker init failed: {e}")
            self.tracker = None

    def reset_tracker(self):
        self.tracker = None
        self.sam_last_bbox = None
        self.sam_last_confidence = 0.0
        self.tracker_frame_counter = 0

    def detect_object_with_sam(self, cv_img):
        ih, iw = cv_img.shape[:2]
        frame_cx, frame_cy = iw / 2, ih / 2
        frame_area = iw * ih

        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        scale = 1.0
        if iw > self.sam_max_width:
            scale = self.sam_max_width / float(iw)
            target_h = int(ih * scale)
            rgb = cv2.resize(rgb, (self.sam_max_width, target_h), interpolation=cv2.INTER_AREA)

        masks = self.mask_generator.generate(rgb)

        if not masks:
            return None, 0.0

        diag = np.sqrt(iw**2 + ih**2)
        best_mask = None
        best_score = -1.0

        for m in masks:
            # Scale coordinates up to original resolution immediately
            x, y, w, h = [int(v / scale) for v in m['bbox']]
            
            # FIXED: Drop giant background segments directly inside the loop loop
            if w * h > self.max_object_area_ratio * frame_area:
                continue

            mask_cx = x + w / 2
            mask_cy = y + h / 2
            dist = np.sqrt((mask_cx - frame_cx)**2 + (mask_cy - frame_cy)**2)
            centrality = 1.0 - (dist / diag)           
            quality = m['predicted_iou']                
            
            # FIXED: Shifted weight to prioritize shape validity over position
            score = 0.15 * centrality + 0.85 * quality   

            if score > best_score:
                best_score = score
                best_mask = m

        if best_mask is None:
            return None, 0.0

        x, y, w, h = [int(v / scale) for v in best_mask['bbox']]
        confidence = float(best_mask['predicted_iou'])
        return (x, y, w, h), confidence

    def synchronized_callback(self, image_msg, tof_msg):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            ih, iw, _ = cv_img.shape
            tof_distance = tof_msg.range

            cx, cy = iw // 2, ih // 2
            cv2.line(cv_img, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 2)
            cv2.line(cv_img, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 2)

            if not self.is_scanning:
                cv2.putText(cv_img, "SYSTEM STATE: STANDBY", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(cv_img, f"ToF Dist: {tof_distance:.2f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow("Sensor Fusion HUD Monitor", cv_img)
                cv2.waitKey(1)
                return

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

            # 2. MobileSAM + tracker fallback with Forced Re-verification
            if frame_type == "none":
                if self.tracker is not None:
                    self.tracker_frame_counter += 1
                    
                    if self.tracker_frame_counter >= self.tracker_update_interval:
                        self.reset_tracker()
                    else:
                        success, tracked_bbox = self.tracker.update(cv_img)
                        if success:
                            target_bbox = tuple(map(int, tracked_bbox))
                            frame_type = "object"
                            frame_confidence = self.sam_last_confidence
                            centered = self.is_spatial_match(target_bbox, iw, ih)
                            self.current_sam_skip = self.sam_active_skip 
                        else:
                            self.reset_tracker()

                if frame_type == "none":
                    self.sam_frame_counter += 1
                    
                    if self.sam_frame_counter >= self.current_sam_skip or self.sam_last_bbox is None:
                        self.sam_frame_counter = 0 
                        self.sam_last_bbox, self.sam_last_confidence = self.detect_object_with_sam(cv_img)

                        if self.sam_last_bbox is not None:
                            self.current_sam_skip = self.sam_active_skip
                            frame_type = "object"
                            frame_confidence = self.sam_last_confidence
                            target_bbox = self.sam_last_bbox
                            centered = self.is_spatial_match(target_bbox, iw, ih)
                            self.init_tracker(cv_img, target_bbox)
                        else:
                            self.current_sam_skip = self.sam_idle_skip

            # 3. HUD Graphics & State Pipeline
            if frame_type != "none" and target_bbox:
                x, y, w, h = target_bbox
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

                cv2.putText(cv_img, "SYSTEM STATE: ALIGNING CHASSIS", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                self.centering_start_time = None
                if centered:
                    self.stop_robot()
                    boost = 0 if frame_type == "face" else 0
                    confidence = np.clip(frame_confidence + boost, 0.0, 1.0)
                    self.state_history.append((frame_type, confidence))
                    self.distance_history.append(tof_distance)
                    cv2.putText(cv_img, "SYSTEM STATE: CAPTURING SNAPSHOT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    self.state_history.append(("none", 1.0))
                    self.distance_history.append(0.0)
                    cv2.putText(cv_img, "SYSTEM STATE: SCANNING (EMPTY)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                progress = len(self.state_history)
                cv2.rectangle(cv_img, (20, 90), (220, 105), (50, 50, 50), -1)
                cv2.rectangle(cv_img, (20, 90), (20 + (progress * 20), 105), (0, 255, 0), -1)
                cv2.putText(cv_img, f"Samples: {progress}/{self.sample_size}", (230, 103), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.putText(cv_img, f"Live ToF Distance: {tof_distance:.2f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow("Sensor Fusion HUD Monitor", cv_img)
            cv2.waitKey(1)

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