#!/usr/bin/env python3

import json
import math
import time
import os
import ast
from pathlib import Path

import rclpy
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data

class Algorithm_utils:
    """Mixin class providing utilities, callbacks, and helpers for AlgorithmNode."""
    
    def wait_for_system_mesh(self):
        """Non-blocking network graph checker wrapper."""
        current_nodes = self.get_node_names()
        missing_nodes = [node for node in self.required_nodes if node not in current_nodes]
        if not missing_nodes:
            return True
        else:
            self.get_logger().warn(f"Waiting for mesh topology nodes: {missing_nodes}", throttle_duration_sec=2.0)
            return False
        
    def verify_active_publishers(self):
        """Ensures sensors are publishing AND consumers (mapping/routing) are listening."""
        hardware_ok = self.count_publishers('/tof/distance') > 0 and self.count_publishers('/imu/yaw') > 0
    
        mapping_node_ready = self.is_mapper_node_online
        movement_node_ready = self.count_subscribers('/cmd_movement') > 0
        
        if not (hardware_ok and mapping_node_ready and movement_node_ready):
            self.get_logger().warn(
                f"Waiting for node subscribers... Map Listener: {mapping_node_ready}, Motion Driver: {movement_node_ready}", 
                throttle_duration_sec=3.0
            )
            return False
            
        return True

    # --- Core ROS 2 Sensor Callbacks ---

    def tof_callback(self, msg):
        self.current_distance = msg.range

    def yaw_callback(self, msg):
        self.current_yaw = msg.data

    def movement_finished_callback(self, msg):
        if msg.data is True:
            self.movement_busy = False
    
    def map_callback(self, msg):
        """Stores the global semantic occupancy grid representation."""
        self.current_map = msg

    def mapping_status_callback(self, msg):
        self.is_mapper_node_online = msg.data

    def tracking_angle_callback(self, msg):
        """Processes continuous numeric arrays from the Object Detection node's tracker."""
        if msg.data:
            self.latest_tracking_angle = msg.data[0]
            self.target_detected_in_frame = True
        else:
            self.target_detected_in_frame = False

    def face_tracking_angle_callback(self, msg):
        """Processes continuous numeric arrays from the Face Recognition node's tracker."""
        if msg.data:
            self.latest_face_tracking_angle = msg.data[0]
            self.face_detected_in_frame = True
        else:
            self.face_detected_in_frame = False

    # --- Active Anti-Stall / Collision Logic ---

    def check_for_motor_stall(self):
        """Monitors IMU updates while moving to catch hidden collisions."""
        # SAFEGUARD 1: Do not monitor stalls if we are only making fine alignment adjustments
        if self.state in [self.StateMachine.SCANNING_ALIGN, self.StateMachine.SCANNING, self.StateMachine.CALLING]:
            self.drive_start_time = None
            self.last_imu_check_time = None
            return False

        if not self.movement_busy:
            self.drive_start_time = None
            self.last_imu_check_time = None
            return False

        # SAFEGUARD 2: Optional micro-turn bypass
        # If the commanded turn is tiny (e.g., less than 5 degrees), ignore stall monitoring
        if hasattr(self, 'current_target_turn_deg') and abs(self.current_target_turn_deg) < 5.0:
            return False

        now = time.time()
        if self.drive_start_time is None:
            self.drive_start_time = now
            self.last_imu_check_time = now
            self.last_tracked_yaw = self.current_yaw
            return False

        if now - self.last_imu_check_time >= 3.0:
            yaw_delta = abs(self.current_yaw - self.last_tracked_yaw)
            if yaw_delta > math.pi:
                yaw_delta = (2.0 * math.pi) - yaw_delta

            if (yaw_delta < math.radians(1.5)) and (now - self.drive_start_time > 4.0):
                self.get_logger().error("STALL DETECTED! Hit unmapped obstacle. Re-routing to FINDGAP.")
                self.stall_detected = False
                self.hit_unmapped_obstacle = False
                if hasattr(self, 'stall_counter'):
                    self.stall_counter = 0
        
                self.transition_to(self.StateMachine.FINDGAP)
                return True
            
            self.last_imu_check_time = now
            self.last_tracked_yaw = self.current_yaw
            
        return False

    # --- Local Map Grid Path Optimization ---

    def find_clear_tracking_angle(self, target_distance_m=0.25):
        """Parses the live OccupancyGrid data to find an obstruction-free relative gap."""
        if self.current_map is None:
            self.get_logger().warn("Map message matrix missing. Defaulting to straight ahead.")
            return 0.0

        grid = self.current_map.data
        res = self.current_map.info.resolution
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        robot_idx = None
        for idx, val in enumerate(grid):
            if val == 1: 
                robot_idx = idx
                break
                
        if robot_idx is not None:
            robot_col = robot_idx % width
            robot_row = robot_idx // width
        else:
            robot_col = int((0.0 - origin_x) / res)
            robot_row = int((0.0 - origin_y) / res)

        best_angle_deg = None
        max_clear_score = -1

        # Sweep a full 360-degree panorama in 15-degree wedges to locate gaps
        for rel_angle in range(-180, 181, 15):
            abs_rad = self.current_yaw + math.radians(rel_angle)
            path_is_blocked = False
            clear_cells_count = 0
            
            steps = int(target_distance_m / res) + 1
            for step in range(1, steps + 1):
                dist = step * res
                test_x = robot_col + int((dist * math.cos(abs_rad)) / res)
                test_y = robot_row + int((dist * math.sin(abs_rad)) / res)
                
                if (0 <= test_x < width) and (0 <= test_y < height):
                    cell_index = test_y * width + test_x
                    cell_value = grid[cell_index]
                    
                    # Values 2, 3, 4 are hazardous obstacles mapped by grid_mapping_node
                    if cell_value > 1: 
                        path_is_blocked = True
                        break
                    else:
                        clear_cells_count += 1
                else:
                    path_is_blocked = True
                    break
            
            if not path_is_blocked and clear_cells_count > max_clear_score:
                max_clear_score = clear_cells_count
                best_angle_deg = float(rel_angle)

        if best_angle_deg is not None:
            self.get_logger().info(f"Selected clear gap relative path: {best_angle_deg}°")
            return best_angle_deg
        
        self.get_logger().error("No escaping paths identified in local map grid! Defaulting spin.")
        return 180.0 # Pivot around entirely if boxed in
    

    def find_next_target(self, min_range_m=0.25, max_range_m=2.0):
        if self.current_map is None:
            self.get_logger().warn("Map message missing for target hunting. Returning None.")
            return None, None

        grid = self.current_map.data
        res = self.current_map.info.resolution
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        robot_idx = None
        for idx, val in enumerate(grid):
            if val == 1: 
                robot_idx = idx
                break
                
        if robot_idx is not None:
            robot_col = robot_idx % width
            robot_row = robot_idx // width
        else:
            robot_col = int((0.0 - origin_x) / res)
            robot_row = int((0.0 - origin_y) / res)

        closest_distant_target = None
        min_target_dist = float('inf')

        for rel_angle in range(-180, 181, 10):
            abs_rad = self.current_yaw + math.radians(rel_angle)
            
            steps = int(max_range_m / res) + 1
            
            for step in range(1, steps + 1):
                dist = step * res
                test_x = robot_col + int((dist * math.cos(abs_rad)) / res)
                test_y = robot_row + int((dist * math.sin(abs_rad)) / res)
                
                if (0 <= test_x < width) and (0 <= test_y < height):
                    cell_index = test_y * width + test_x
                    cell_value = grid[cell_index]
                    
                    if cell_value > 1:
                        if min_range_m <= dist <= max_range_m:
                            if dist < min_target_dist:
                                min_target_dist = dist
                                closest_distant_target = (float(dist), float(rel_angle))
                        
                        break
                else:
                    break

        if closest_distant_target is not None:
            target_distance, target_heading = closest_distant_target
            self.get_logger().info(f"Found distant target at Dist: {target_distance}m, Heading: {target_heading}°")
            return target_distance, target_heading

        self.get_logger().info("No distant objects detected within range parameters.")
        return 0.0, 0.0

    # --- Sequential Logic Recovery State Controls ---

    def clean_alignment_and_resume(self):
        """Reverts total alignment drift offset to return the chassis to the scan sweep line."""
        if self.has_aligned:
            self.get_logger().info(f"Reverting total alignment drift offset: {-self.alignment_offset_deg:.2f}°")
            self.send_drive_command(0.0, -self.alignment_offset_deg)
            self.has_aligned = False
            self.alignment_offset_deg = 0.0
        self.resume_search_sequence()

    def resume_search_sequence(self):
        """Resets structural scan tracking sub-states and re-enters search loop states."""
        self.scan_triggered = False
        self.object_scan_done = False
        self.face_scan_done = False
        
        if self.scan_target_max == 90.0:
            self.transition_to(self.StateMachine.SEARCHING_INIT)
        else:
            self.transition_to(self.StateMachine.SEARCHING)

    # --- Command Utilities ---

    def transition_from_to(self, next_state):
        self.get_logger().info(f"State Transition: {self.state.name} -> {next_state.name}")
        self.previous_state = self.state
        self.state = next_state

    

    def send_drive_command(self, distance, angle_deg):

        msg = Float32MultiArray()
        msg.data = [float(distance), float(angle_deg)]
        self.cmd_pub.publish(msg)
        self.movement_busy = True