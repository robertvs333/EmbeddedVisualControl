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
        """Ensures underlying topics have active hardware streams."""
        return self.count_publishers('/tof/distance') > 0 and self.count_publishers('/imu/yaw') > 0

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
        if not self.movement_busy:
            self.drive_start_time = None
            self.last_imu_check_time = None
            return False

        now = time.time()
        if self.drive_start_time is None:
            self.drive_start_time = now
            self.last_imu_check_time = now
            self.last_tracked_yaw = self.current_yaw
            return False

        if now - self.last_imu_check_time >= 2.0:
            yaw_delta = abs(self.current_yaw - self.last_tracked_yaw)
            if yaw_delta > math.pi:
                yaw_delta = (2.0 * math.pi) - yaw_delta

            if (yaw_delta < math.radians(1.5)) and (now - self.drive_start_time > 4.0):
                self.get_logger().error("STALL DETECTED! Hit unmapped obstacle. Re-routing to FINDGAP.")
                self.drive_start_time = None
                self.last_imu_check_time = None
                
                self.motor.set_wheels_speed(0.0, 0.0)
                self.transition_to(self.StateMachine.FINDGAP)
                return True
            
            self.last_imu_check_time = now
            self.last_tracked_yaw = self.current_yaw
        return False

    # --- Local Map Grid Path Optimization ---

    def find_clear_tracking_angle(self, target_distance_m=0.25):
        """Parses the current OccupancyGrid to find a clear path orientation."""
        if self.current_map is None:
            self.get_logger().warn("Map array missing. Defaulting to straight ahead.")
            return 0.0

        grid = self.current_map.data
        res = self.current_map.info.resolution
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        robot_x_m = self.current_map.info.origin.position.x
        robot_y_m = self.current_map.info.origin.position.y
        
        robot_col = int((0.0 - robot_x_m) / res)
        robot_row = int((0.0 - robot_y_m) / res)

        best_angle_deg = None
        max_clear_score = -1

        for rel_angle in range(-90, 91, 15):
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
                    
                    if cell_value > 1: # Values 2,3,4 are mapped structural hazards
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
            self.get_logger().info(f"Selected clear path angle: {best_angle_deg}° relative.")
            return best_angle_deg
        
        self.get_logger().error("No clear paths found in map! Defaulting straight.")
        return 0.0

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

    def transition_to(self, next_state):
        self.get_logger().info(f"State Transition: {self.state.name} -> {next_state.name}")
        self.state = next_state

    def send_drive_command(self, distance, angle_deg):
        msg = Float32MultiArray()
        msg.data = [float(distance), float(angle_deg)]
        self.cmd_pub.publish(msg)
        self.movement_busy = True