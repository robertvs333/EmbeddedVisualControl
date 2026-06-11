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
    def __init__(self):
        super().__init__('camera_detection')
        self.bridge = CvBridge()

        # 1. Initialize DPT Hybrid Engine
        self.get_logger().info("Initializing DPT Hybrid Depth Engine...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load DPT_Hybrid
        self.model = torch.hub.load("intel-isl/MiDaS", "DPT_Hybrid")
        self.transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = self.transforms.dpt_transform
        
        self.model.to(self.device)
        self.model.eval()

        # 2. Parameters
        self.camera_fov_rad = 1.05
        self.depth_proximity_gate = 80
        self.is_scanning = False
        # Reduced sample size to compensate for DPT_Hybrid's heavier compute requirement
        self.sample_size = 3 
        self.state_history = [] 

        # 4. ROS 2 Communication
        self.trigger_sub = self.create_subscription(String, '/detection/trigger', self.trigger_callback, 10)
        self.result_pub  = self.create_publisher(String, '/detection/final_result', 10)
        self.angles_pub  = self.create_publisher(Float32MultiArray, '/detection/tracking_angles', 10)
        self.image_sub   = self.create_subscription(
            CompressedImage, '/camera/image_undistorted',
            self.image_callback, qos_profile=qos_profile_sensor_data)

        cv2.namedWindow("Sensor Fusion Diagnostic Desk", cv2.WINDOW_NORMAL)
        self.get_logger().info("Multi-Object Depth Detection Node (DPT_Hybrid) Initialized.")

    def _get_depth_map(self, cv_img):
        """Run DPT_Hybrid; returns uint8 map."""
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        input_tensor = self.transform(img_rgb).to(self.device)
        
        with torch.no_grad():
            depth = self.model(input_tensor)
            depth = torch.nn.functional.interpolate(
                depth.unsqueeze(1), size=cv_img.shape[:2],
                mode="bicubic", align_corners=False).squeeze()
        
        depth_np = cv2.normalize(depth.cpu().numpy(), None, 0, 255, cv2.NORM_MINMAX)
        return depth_np.astype(np.uint8)

    def _depth_blobs(self, depth_map):
        #ih, iw = depth_map.shape
        #mask_height = int(ih * 0.1) 
        #depth_map[0:mask_height, :] = 0  # Zero out the top 40% of the image
        # 1. Background subtraction
        floor_estimate = cv2.GaussianBlur(depth_map, (91, 91), 0)
        relative_depth = cv2.subtract(depth_map, floor_estimate)

        # 2. Thresholding
        _, thresh = cv2.threshold(relative_depth, 10, 255, cv2.THRESH_BINARY)
        
        # 3. Distance Transform (The "Global View" logic)
        # This creates a map where pixels further from the edge have higher values
        dist = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        
        # 4. Find the "peak" of the objects
        # We only look for the absolute center of each protruding blob
        _, _, _, max_loc = cv2.minMaxLoc(dist)
        
        # We use a threshold on the distance map to capture the full object
        # instead of just the noisy edges.
        _, object_mask = cv2.threshold(dist, dist.max() * 0.2, 255, cv2.THRESH_BINARY)
        object_mask = object_mask.astype(np.uint8)

        # 5. Extract contours from the distance-based mask
        contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        raw_boxes = []
        depth_values = []
        for c in contours:
            if cv2.contourArea(c) > 500: # Filter small noise
                raw_boxes.append(cv2.boundingRect(c))
                # Sample depth based on the mask
                mask_roi = np.zeros_like(depth_map)
                cv2.drawContours(mask_roi, [c], -1, 255, -1)
                depth_values.append(float(cv2.mean(depth_map, mask=mask_roi)[0]))

        return raw_boxes, depth_values, object_mask, relative_depth

    def _merge_nearby_boxes(self, boxes, threshold=60):
        """Merges boxes that are spatially close."""
        if not boxes: return []
        
        # Simple clustering: if boxes overlap or are within distance, merge them
        merged = []
        while boxes:
            curr = boxes.pop(0)
            merged_with_curr = [curr]
            for i in range(len(boxes) - 1, -1, -1):
                # Calculate distance between centers
                c1 = (curr[0] + curr[2]/2, curr[1] + curr[3]/2)
                c2 = (boxes[i][0] + boxes[i][2]/2, boxes[i][1] + boxes[i][3]/2)
                if np.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) < threshold:
                    merged_with_curr.append(boxes.pop(i))
            
            # Combine into one big box
            min_x = min(b[0] for b in merged_with_curr)
            min_y = min(b[1] for b in merged_with_curr)
            max_x = max(b[0] + b[2] for b in merged_with_curr)
            max_y = max(b[1] + b[3] for b in merged_with_curr)
            merged.append((min_x, min_y, max_x - min_x, max_y - min_y))
        return merged

    # -----------------------------------------------------------------------
    # ROS CALLBACKS
    # -----------------------------------------------------------------------

    def trigger_callback(self, msg):
        if self.is_scanning:
            return
        self.get_logger().info("Scan triggered!")
        self.state_history.clear()
        self.is_scanning = True
    

    def image_callback(self, image_msg):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(image_msg, 'bgr8')
            ih, iw = cv_img.shape[:2]

            if not self.is_scanning:
                cv2.putText(cv_img, "SYS STATE: STANDBY", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Sensor Fusion Diagnostic Desk", cv_img)
                cv2.waitKey(1)
                return

            # --- DETECTION ---
            depth_map = self._get_depth_map(cv_img)
            boxes, depth_values, vis_mask, relative_depth = self._depth_blobs(depth_map)

            # --- ANGLES + PER-FRAME PUBLISH ---
            # Per-frame payload: flat array [angle0, depth0, angle1, depth1, ...]
            frame_objects = []
            flat_payload  = []
            cv2.line(cv_img, (iw // 2, 0), (iw // 2, ih), (255, 255, 0), 1)

            for (x, y, w, h), d in zip(boxes, depth_values):
                angle = (0.5 - ((x + w / 2) / iw)) * self.camera_fov_rad
                frame_objects.append({"angle": round(angle, 4), "depth": round(d, 1)})
                flat_payload.extend([float(angle), float(d)])

                # HUD
                hud = (0, 220, 80)
                cv2.rectangle(cv_img, (x, y), (x+w, y+h), hud, 2)
                cv2.circle(cv_img, (int(x + w/2), int(y + h/2)), 4, hud, -1)
                cv2.putText(cv_img, f"A:{angle:.2f} D:{d:.0f}",
                            (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, hud, 1)

            cv2.putText(cv_img, f"DETECTED: {len(boxes)}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            self.angles_pub.publish(Float32MultiArray(data=flat_payload))
            self.state_history.append(frame_objects)

            # --- DIAGNOSTIC PANORAMA ---
            rel_vis = cv2.cvtColor(
                cv2.normalize(relative_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
                cv2.COLOR_GRAY2BGR)

            det_vis = cv_img.copy()
            for (x, y, w, h) in boxes:
                cv2.rectangle(det_vis, (x, y), (x+w, y+h), (0, 220, 80), 3)

            panels_data = [
                (cv_img,                                             "1. CAM + HUD"),
                (cv2.applyColorMap(depth_map, cv2.COLORMAP_INFERNO),"2. DEPTH MAP"),
                (rel_vis,                                            "3. RELATIVE DEPTH"),
                (cv2.cvtColor(vis_mask, cv2.COLOR_GRAY2BGR),        "4. WATERSHED MASK"),
                (det_vis,                                            "5. DETECTIONS"),
            ]
            panels = []
            for img, label in panels_data:
                p = img.copy()
                cv2.putText(p, label, (10, ih - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                panels.append(p)

            cv2.imshow("Sensor Fusion Diagnostic Desk",
                       cv2.resize(np.hstack(panels), (2000, 400)))
            cv2.waitKey(1)

            if len(self.state_history) >= self.sample_size:
                self.evaluate_and_publish_final_result()

        except Exception as e:
            self.get_logger().error(f"Pipeline error: {e}")
            self.is_scanning = False

    # -----------------------------------------------------------------------
    # FINAL RESULT
    # -----------------------------------------------------------------------

    def evaluate_and_publish_final_result(self):
        self.is_scanning = False
        
        # 1. Flatten all detections from all frames into a list of [angle, depth]
        all_detections = [obj for frame in self.state_history for obj in frame]
        if not all_detections:
            return self.result_pub.publish(String(data=json.dumps({"status": "no_objects"})))

        # 2. Consolidation thresholds (tune these!)
        angle_thresh = 0.15  # Radians
        depth_thresh = 30.0  # Units
        
        final_objects = []
        
        # 3. Consolidate
        for det in all_detections:
            found_match = False
            for obj in final_objects:
                diff_angle = abs(det["angle"] - obj["angle"])
                diff_depth = abs(det["depth"] - obj["depth"])
                
                if diff_angle < angle_thresh and diff_depth < depth_thresh:
                    # They are the same object, keep the average
                    obj["angle"] = (obj["angle"] + det["angle"]) / 2
                    obj["depth"] = (obj["depth"] + det["depth"]) / 2
                    found_match = True
                    break
            
            if not found_match:
                final_objects.append(det)

        output_msg = String(data=json.dumps({
            "status": "scan_complete", 
            "detected_object_count": len(final_objects),
            "objects": final_objects
        }))
        self.result_pub.publish(output_msg)
        self.get_logger().info(f"Scan complete: {output_msg.data}")


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

if __name__ == '__main__':
    main()