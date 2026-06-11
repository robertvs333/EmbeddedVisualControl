#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray

import cv2
from cv_bridge import CvBridge
import numpy as np
import torch
import os
import json

class MultiObjectDetectionNode(Node):
    def _init_(self):
        super()._init_('camera_detection')
        self.bridge = CvBridge()

        # 1. Initialize MiDaS Engine (Used STRICTLY as a coarse background filter)
        self.get_logger().info("Initializing MiDaS Coarse Spatial Gate...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.midas.to(self.device)
        self.midas.eval()

        # 2. Vision & Control Tuning Parameters
        self.camera_fov_rad = 1.05
        
        # --- GRADIENT ENGINE PARAMETERS ---
        self.gradient_threshold = 35          
        self.depth_proximity_gate = 65        
        
        self.is_scanning = False
        self.sample_size = 20          
        self.state_history = []  # Stores lists of angles detected per frame

        # 3. Load Lens Cast Calibration (LCC) Map
        self.calib_file = "lens_calibration.npy"
        self.calibration_map = None
        if os.path.exists(self.calib_file):
            self.calibration_map = np.load(self.calib_file)

        # 4. ROS 2 Communication Architecture
        self.trigger_sub = self.create_subscription(String, '/detection/trigger', self.trigger_callback, 10)
        self.result_pub = self.create_publisher(String, '/detection/final_result', 10)
        self.angles_pub = self.create_publisher(Float32MultiArray, '/detection/tracking_angles', 10)
        
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_undistorted',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )

        cv2.namedWindow("Sensor Fusion Diagnostic Desk", cv2.WINDOW_NORMAL)
        self.get_logger().info("Multi-Object Detection Node Initialized.")

    def _apply_calibration(self, img):
        if self.calibration_map is None:
            return img
        return np.clip(img.astype(np.float32) * self.calibration_map, 0, 255).astype(np.uint8)

    def trigger_callback(self, msg):
        if self.is_scanning:
            return
        self.get_logger().info("Scan command triggered for multiple objects!")
        self.state_history.clear()
        self.is_scanning = True

    def image_callback(self, image_msg):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_img = self._apply_calibration(cv_img)
            ih, iw = cv_img.shape[:2]

            if not self.is_scanning:
                cv2.putText(cv_img, "SYS STATE: STANDBY", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Sensor Fusion Diagnostic Desk", cv_img)
                cv2.waitKey(1)
                return

            # --- STEP 1: SOBEL-SCHARR DIRECTIONAL GRADIENT ENGINE ---
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            
            grad_x = cv2.Scharr(blurred, cv2.CV_16S, 1, 0)
            grad_y = cv2.Scharr(blurred, cv2.CV_16S, 0, 1)
            
            abs_grad_x = cv2.convertScaleAbs(grad_x)
            abs_grad_y = cv2.convertScaleAbs(grad_y)
            
            gradient_magnitude = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
            _, saliency_mask = cv2.threshold(gradient_magnitude, self.gradient_threshold, 255, cv2.THRESH_BINARY)
            
            g_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (13, 13))
            saliency_mask = cv2.morphologyEx(saliency_mask, cv2.MORPH_CLOSE, g_kernel)

            # --- STEP 2: COARSE MIDAS DEPTH SPATIAL GATE ---
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            input_batch = self.midas_transforms.small_transform(img_rgb).to(self.device)
            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(prediction.unsqueeze(1), size=(ih, iw), mode="bicubic", align_corners=False).squeeze()
            depth_normalized = cv2.normalize(prediction.cpu().numpy(), None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            proximity_mask = (depth_normalized > self.depth_proximity_gate).astype(np.uint8) * 255
            saliency_mask = 255 - saliency_mask

            # --- STEP 3: MASK INTERSECTION FUSION ---
            fused_mask = cv2.bitwise_and(saliency_mask, proximity_mask)
            fused_mask = cv2.morphologyEx(fused_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # --- STEP 4: MULTI-CONTOUR CENTER ANALYSIS ---
            contours, _ = cv2.findContours(fused_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            current_frame_angles = []
            cv2.line(cv_img, (int(iw/2), 0), (int(iw/2), ih), (255, 255, 0), 1, cv2.LINE_AA)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 400:  # Matches your minimum item filter constraints
                    x, y, w, h = cv2.boundingRect(contour)
                    bbox_center_x = x + (w / 2)
                    
                    normalized_error = 0.5 - (bbox_center_x / iw)
                    target_angle = normalized_error * self.camera_fov_rad
                    current_frame_angles.append(float(target_angle))

                    # Visualizations per object
                    hud_color = (0, 165, 255)
                    cv2.rectangle(cv_img, (x, y), (x + w, y + h), hud_color, 2)
                    cv2.circle(cv_img, (int(bbox_center_x), int(y + h/2)), 5, hud_color, -1)
                    cv2.putText(cv_img, f"ANG: {target_angle:.2f}R", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, hud_color, 1)

            # Publish the matrix array of tracking angles for this specific frame
            angles_msg = Float32MultiArray()
            angles_msg.data = current_frame_angles
            self.angles_pub.publish(angles_msg)

            # Log frame history
            self.state_history.append(current_frame_angles)

            # Display total count overhead
            cv2.putText(cv_img, f"OBJECTS DETECTED: {len(current_frame_angles)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # --- RENDERING PANORAMIC DIAGNOSTIC DESK ---
            object_visual = cv2.cvtColor(saliency_mask, cv2.COLOR_GRAY2BGR)
            prox_visual   = cv2.cvtColor(proximity_mask, cv2.COLOR_GRAY2BGR)
            fused_visual  = cv2.cvtColor(fused_mask, cv2.COLOR_GRAY2BGR)

            cv2.putText(cv_img,        "1. CAM VIEW",    (15, ih - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(object_visual, "2. SCHARR GRAD", (15, ih - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(prox_visual,   "3. COARSE DEPTH",(15, ih - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(fused_visual,  "4. FUSED FINAL", (15, ih - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            diagnostic_panorama = np.hstack((cv_img, object_visual, prox_visual, fused_visual))
            cv2.imshow("Sensor Fusion Diagnostic Desk", diagnostic_panorama)
            cv2.waitKey(1)

            if len(self.state_history) >= self.sample_size:
                self.evaluate_and_publish_final_result()

        except Exception as e:
            self.get_logger().error(f"Multi-Object Pipeline Broken: {e}")
            self.is_scanning = False

    def evaluate_and_publish_final_result(self):
        self.is_scanning = False
        
        # 1. Flatten the frame history into a single list of all detected raw angles
        all_detected_angles = [angle for frame in self.state_history for angle in frame]
        
        final_object_angles = []
        
        if all_detected_angles:
            # Calculate the average number of distinct objects spotted per frame
            counts = [len(frame) for frame in self.state_history if len(frame) > 0]
            avg_objects = int(np.round(np.mean(counts))) if counts else 0
            
            # Convert raw angles to a 2D float32 array for OpenCV's KMeans
            data = np.array(all_detected_angles, dtype=np.float32).reshape(-1, 1)
            
            if avg_objects > 0 and len(data) >= avg_objects:
                # Define KMeans criteria (Stop if it converges within 1.0 accuracy or hits 10 iterations)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
                
                # Run KMeans to sort the data into 'avg_objects' number of clusters
                compactness, labels, centers = cv2.kmeans(
                    data, 
                    avg_objects, 
                    None, 
                    criteria, 
                    10, 
                    cv2.KMEANS_RANDOM_CENTERS
                )
                
                # Centers contains the exact average angle for each object group
                final_object_angles = [float(c[0]) for c in centers]
                # Sort them from left-to-right (negative to positive angles) for consistency
                final_object_angles.sort()
            else:
                # Fallback if object tracking was completely erratic
                final_object_angles = [float(np.mean(all_detected_angles))]
        else:
            avg_objects = 0

        # 2. Build and publish clean JSON payload
        output_data = {
            "status": "scan_complete",
            "detected_object_count": len(final_object_angles),
            "final_averaged_angles": [round(a, 4) for a in final_object_angles]
        }

        output_msg = String()
        output_msg.data = json.dumps(output_data)
        self.result_pub.publish(output_msg)
        self.get_logger().info(f"Scan Loop Terminated. Multi-Object Payload: {output_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if _name_ == '_main_':
    main()