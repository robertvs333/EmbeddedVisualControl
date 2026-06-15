#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
from enum import Enum, auto
import json

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver
from jetson_camera.algorithm_util import Algorithm_utils

class AlgorithmNode(Node, Algorithm_utils):

    class StateMachine(Enum):
        BOOTING = auto()
        INIT = auto()
        SEARCHING_INIT = auto()
        SEARCHING = auto()
        SCANNING = auto()
        SCANNING_ALIGN = auto()  
        SCANNING_CAPTURE = auto()
        FINDGAP = auto()
        TRACKING = auto()
        CALLING = auto()

    def __init__(self):
        super().__init__('algorithm_node')
        
        self.required_nodes = [
            'encoder_node',
            'imu_node',
            'grid_mapping_node',
            'movement_node',
            'ToF_sensor',
            'route_plotter',
        ]
        
        self.is_robot_ready = False
        self.motor = DaguWheelsDriver()

        # Internal State Variables
        self.face_found_globally = False
        self.state = self.StateMachine.BOOTING
        self.current_distance = None  
        self.current_yaw = 0.0
        self.movement_busy = False    
        self.current_map = None
        self.gap_recovery_triggered = False
        self.is_mapper_node_online = False
        self.sweep_start_yaw = None
        
        # Tracking angle registries
        self.latest_tracking_angle = 0.0
        self.target_detected_in_frame = False
        self.latest_face_tracking_angle = 0.0
        self.face_detected_in_frame = False
        self.gap_command_issued = False

        # Sequential State Scopes
        self.object_scan_done = False
        self.face_scan_done = False
        self.scan_triggered = False
        self.has_aligned = False
        self.alignment_offset_deg = 0.0
        self.alignment_done = False # Added missing flag init
        
        # Structural parameters
        self.scan_total_angle = 0.0
        self.scan_target_max = 90.0   # 90
        self.scan_step_deg = 35.0     
        
        # Safe Driving Stall Check Metrics
        self.last_imu_check_time = None
        self.last_tracked_yaw = 0.0
        self.drive_start_time = None
        self.gap_search_start_time = None

        # --- ROS 2 Publishers & Subscribers ---
        mapping_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_movement', 10)
        self.object_trigger_pub = self.create_publisher(String, '/detection/trigger', 10)
        self.face_trigger_pub = self.create_publisher(String, '/detection/face_trigger', 10)
        self.map_pub = self.create_publisher(String, '/detection/final_result', qos_profile=mapping_qos)
        matching_latching_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(Range, '/tof/distance', self.tof_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_callback, 10)
        self.create_subscription(Bool, '/movement_finished', self.movement_finished_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/tracking_angles', self.tracking_angle_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/face_tracking_angles', self.face_tracking_angle_callback, 10)
        self.create_subscription(OccupancyGrid, '/mapping/semantic_grid', self.map_callback, 10)
        self.mapping_status_sub = self.create_subscription(Bool,'/mapping/active', self.mapping_status_callback, qos_profile=matching_latching_qos)
        self.final_result_sub = self.create_subscription(Bool, '/detection/final_result', self.final_result_callback, 10)
        self.create_subscription(String, '/detection/final_result', self.object_result_callback, 10)
        self.create_subscription(String, '/detection/face_result', self.face_result_callback, 10)

        # Heartbeat clock processing loop
        self.timer = self.create_timer(0.05, self.movement_loop)

    def transition_to(self, next_state):
        self.get_logger().info(f"State Transition: {self.state.name} -> {next_state.name}")
        stationary_states = [
            self.StateMachine.SCANNING_CAPTURE, 
            self.StateMachine.BOOTING, 
            self.StateMachine.CALLING
        ]
        if next_state in stationary_states:
            self.movement_busy = False
            self.drive_start_time = None
            self.last_imu_check_time = None
            
        self.state = next_state

    def object_result_callback(self, msg):
        """Processes Tier 1 outputs from MultiObjectDetectionNode."""
        allowed_states = [self.StateMachine.SCANNING_CAPTURE, self.StateMachine.SCANNING_ALIGN]
        if self.state not in allowed_states or self.object_scan_done:
            return
            
        try:
            payload = json.loads(msg.data.replace("'", '"'))
            
            if payload.get("status") == "scan_complete" and payload.get("detected_object_count", 0) > 0:
                primary_obj = payload["objects"][0]
                
                if not self.has_aligned:
                    self.latest_tracking_angle = primary_obj["angle"]
                    self.target_detected_in_frame = True
                    self.get_logger().info(f"Object detected at offset {self.latest_tracking_angle:.2f}. Running alignment turn...")
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return
                
                live_angle = primary_obj.get("angle", 0.0)
                raw_depth = primary_obj.get("depth", 30.0)
                live_distance = float(raw_depth) / 100.0 if raw_depth > 5.0 else float(raw_depth)

                self.process_and_publish_map_entry(
                    target_type="object", 
                    distance=live_distance, 
                    angle=live_angle
                )
                
                self.object_scan_done = True
                self.scan_triggered = False
                
        except Exception as e:
            self.get_logger().error(f"Error parsing object payload: {e}")
            self.advance_sweep_sequence()

    def advance_sweep_sequence(self):
        """Increments physical search angles, issues rotation steps, and safely yields control."""
        self.scan_triggered = False
        self.object_scan_done = False
        self.face_scan_done = False
        self.target_detected_in_frame = False
        self.alignment_done = False
        
        self.scan_total_angle += self.scan_step_deg
        
        if self.scan_total_angle < self.scan_target_max:
            self.get_logger().info(f"Advancing sweep window. Turning next step: +{self.scan_step_deg}° (Total: {self.scan_total_angle}°/{self.scan_target_max}°)")
            self.send_drive_command(0.0, self.scan_step_deg) 
            self.movement_busy = True  
            # Force transition to SCANNING to preserve state and avoid SEARCHING_INIT instant loopback overrides
            self.transition_to(self.StateMachine.SCANNING)
        else:
            self.get_logger().warn("Full target sweep workspace area completed. Resetting bounds.")
            self.scan_total_angle = 0.0
            self.transition_to(self.StateMachine.FINDGAP)

    def face_result_callback(self, msg):
        """Processes final summary outputs from FaceRecognitionNode."""
        allowed_states = [self.StateMachine.SCANNING_CAPTURE, self.StateMachine.SCANNING_ALIGN]
        if self.state not in allowed_states:
            return
        self.get_logger().info("face detection triggered")

        try:
            payload = json.loads(msg.data.replace("'", '"'))
            identity = payload.get("identity", "Unknown")
            self.get_logger().info(f"identity {identity}")
            
            live_angle = payload.get("angle", payload.get("angle_deg", getattr(self, 'latest_face_tracking_angle', 0.0)))
            
            allowed_identities = ["trump", "Markie", "Musk", "Geert"]
            
            if identity in allowed_identities:
                self.get_logger().info(f"SUCCESS: Face verified! Identity -> [{identity}].")
                self.face_found_globally = True
                
                if not self.has_aligned:
                    self.get_logger().info(f"Face identified [{identity}], executing final precision centering adjustment.")
                    self.latest_tracking_angle = live_angle
                    self.target_detected_in_frame = True
                    self.alignment_done = False
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return # Exit cleanly to let the movement node finish the turn

                self.process_and_publish_map_entry(
                    target_type=f"{identity}", 
                    confidence=1.0,
                    angle=live_angle
                )
                self.face_scan_done = True
                self.advance_sweep_sequence()
                return
            
            else:
                if self.has_aligned:
                    self.process_and_publish_map_entry(
                        target_type="unknown_person", 
                        confidence=1.0,
                        angle=live_angle
                    )
                    self.face_scan_done = True
                    self.advance_sweep_sequence()
                else:
                    self.get_logger().info("Face scan complete: No known faces identified.")
                    self.face_scan_done = True
                    self.advance_sweep_sequence()
            
        except Exception as e:
            self.get_logger().error(f"Error parsing face summary payload: {e}")
            self.advance_sweep_sequence()

    def process_and_publish_map_entry(self, object_payload=None, target_type=None, **kwargs):
        try:
            if object_payload is None:
                object_payload = {}

            raw_type = target_type if target_type is not None else object_payload.get('type')
            if not raw_type:
                raw_type = object_payload.get('_type', 'unknown')
            
            distance = object_payload.get('distance_m', object_payload.get('distance', kwargs.get('distance', None)))
            
            angle_rad = object_payload.get('angle_rad', None)
            if angle_rad is None:
                angle_deg = object_payload.get('angle_deg', object_payload.get('angle', kwargs.get('angle', None)))
                if angle_deg is not None:
                    angle_rad = math.radians(float(angle_deg))

            if distance is not None and angle_rad is not None:
                dist_val = float(distance)
                angle_val = float(angle_rad)
                
                target_x = dist_val * math.cos(angle_val)
                target_y = dist_val * math.sin(angle_val)
                
                MAP_LIMIT_METERS = 1.0
                if abs(target_x) >= MAP_LIMIT_METERS or abs(target_y) >= MAP_LIMIT_METERS:
                    self.get_logger().warn(
                        f"ALGORITHM GUARD: Dropping '{raw_type}' "
                        f"at relative X={target_x:.2f}m, Y={target_y:.2f}m. "
                        f"Exceeds {MAP_LIMIT_METERS}m grid boundaries."
                    )

                    self.face_scan_done = True
                    if hasattr(self, 'advance_sweep_sequence'):
                        self.advance_sweep_sequence()
                    return  # Terminate early before publishing bad values

            payload_dict = {
                "_type": str(raw_type)
            }

            if distance is not None:
                payload_dict["distance_m"] = float(distance)
                
            if angle_rad is not None and float(angle_rad) != 0.0:
                payload_dict["angle_rad"] = float(angle_rad)

            msg = String()
            msg.data = json.dumps(payload_dict)
            
            if hasattr(self, 'map_pub') and self.map_pub is not None:
                self.map_pub.publish(msg)
                self.get_logger().info(f"Published to grid mapper: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error inside process_and_publish_map_entry: {e}")
            # Fallback path to preserve lifecycle workflow on unexpected parsing exceptions
            self.face_scan_done = True
            if hasattr(self, 'advance_sweep_sequence'):
                self.advance_sweep_sequence()

    def final_result_callback(self, msg):
        """Callback to receive finish tracking confirmations from sensory layers."""
        if msg.data:
            self.get_logger().info("Received final vision detection signal. Breaking out of SCANNING_CAPTURE.")
            self.detection_received = True

            if self.state == self.StateMachine.SCANNING_CAPTURE:
                self.transition_to(self.StateMachine.SCANNING)

    def movement_loop(self):
        if self.check_for_motor_stall():
            return
        if self.movement_busy:
            return

        # BOOTING state
        if self.state == self.StateMachine.BOOTING:
            if self.wait_for_system_mesh():
                if self.verify_active_publishers():
                    self.is_robot_ready = True
                    self.transition_to(self.StateMachine.INIT)  

        # INIT state
        elif self.state == self.StateMachine.INIT:
            if self.verify_active_publishers():
                if self.current_distance is not None and self.current_yaw is not None:
                    self.get_logger().info("Sensors settled. Core data feeds initialized.")
                    self.transition_to(self.StateMachine.SEARCHING_INIT)

        # SEARCH INIT STATE 
        elif self.state == self.StateMachine.SEARCHING_INIT:
            if self.face_found_globally:
                self.get_logger().info("Target face confirmed globally. Proceeding to TRACKING.")
                self.sweep_start_yaw = None 
                self.gap_command_issued = False
                self.transition_to(self.StateMachine.FINDGAP)
                return

            if self.sweep_start_yaw is None:
                self.sweep_start_yaw = self.current_yaw
                self.get_logger().info(f"Locking sweep baseline yaw at: {math.degrees(self.sweep_start_yaw):.2f}°")

            yaw_delta = abs(self.current_yaw - self.sweep_start_yaw)
            if yaw_delta > math.pi:
                yaw_delta = (2.0 * math.pi) - yaw_delta
                
            actual_rotation_deg = math.degrees(yaw_delta)

            if actual_rotation_deg >= self.scan_target_max:
                self.get_logger().warn(f"Sweep limit reached ({actual_rotation_deg:.1f}° >= {self.scan_target_max}°). Repeating sweep.")
                self.sweep_start_yaw = None 
                self.transition_to(self.StateMachine.SCANNING)
            else:
                self.has_aligned = False  
                self.object_scan_done = False
                self.face_scan_done = False
                self.scan_triggered = False
                self.transition_to(self.StateMachine.SCANNING_CAPTURE)

        # SCANNING: 
        elif self.state == self.StateMachine.SCANNING:
            self.has_aligned = False  
            self.object_scan_done = False
            self.face_scan_done = False
            self.scan_triggered = False
            
            # This state acts as an active physical motor movement block step tracker
            self.get_logger().info(f"Sweeping: Advancing step turn of {self.scan_step_deg}°")
            self.send_drive_command(0.0, self.scan_step_deg)
            self.movement_busy = True # Ensure timer pauses while movement executes
            self.transition_to(self.StateMachine.SCANNING_CAPTURE)

        # SCANNING CAPTURE:
        elif self.state == self.StateMachine.SCANNING_CAPTURE:
            if not self.object_scan_done:
                if not self.scan_triggered:
                    self.get_logger().info("Triggering Multi-Object Detection...")
                    msg = String(data="START_SCAN")
                    self.object_trigger_pub.publish(msg)
                    self.scan_triggered = True
                return 

            elif not self.face_scan_done:
                if not self.scan_triggered:
                    self.get_logger().info("Triggering Face Detection on verified target...")
                    msg = String(data="START_SCAN")
                    self.face_trigger_pub.publish(msg)
                    self.scan_triggered = True
                return
                
            elif self.face_scan_done:
                self.get_logger().info("Face captured completely. Advancing process.")
                self.transition_to(self.StateMachine.CALLING)

        # SCANNING ALIGN: 
        elif self.state == self.StateMachine.SCANNING_ALIGN:
            if self.target_detected_in_frame:
                step_offset = math.degrees(self.latest_tracking_angle)
                self.current_target_turn_deg = step_offset
                self.get_logger().info(f"Aligning camera axis. Turn offset: {step_offset:.2f}°")
                
                self.has_aligned = True
                self.movement_busy = True
                self.alignment_offset_deg += step_offset 
                self.send_drive_command(0.0, -step_offset)

                self.scan_triggered = False 
                self.target_detected_in_frame = False
                self.face_detected_in_frame = False
                self.alignment_done = True
                self.transition_to(self.StateMachine.SCANNING_CAPTURE)
            else:
                self.get_logger().warn("Alignment target lost frame reference. Resuming search loop.")
                self.clean_alignment_and_resume()

        # TRACKING:
        elif self.state == self.StateMachine.TRACKING:
            self.get_logger().info("TRACKING: Evaluating map to find clear path...")
            
            target_distance = 0.0
            target_heading = 0.0
            
            if not self.gap_command_issued:
                target_distance, target_heading = self.find_next_target()
                if target_distance is None or target_heading is None:
                    target_distance, target_heading = 0.0, 0.0
                self.send_drive_command(target_distance, target_heading)
    
            elif self.gap_command_issued:
                escape_heading = self.find_clear_tracking_angle(target_distance_m=0.4)
                if escape_heading is None:
                    escape_heading = 0.0
                self.best_gap_angle = escape_heading
                self.send_drive_command(0.35, escape_heading)
            
            self.scan_total_angle = 0.0
            self.scan_target_max = 180.0 
            self.transition_to(self.StateMachine.SEARCHING)

        # SEARCHING 
        elif self.state == self.StateMachine.SEARCHING:
            if abs(self.scan_total_angle) >= self.scan_target_max:
                if self.face_found_globally:
                    self.get_logger().info("Wide area SEARCHING complete and face verified. Moving to CALLING.")
                    self.transition_to(self.StateMachine.CALLING)
                else:
                    self.get_logger().warn("Wide sweep completed but face reference is missing. Re-running wide area sweep loop.")
                    self.scan_total_angle = 0.0 
                    self.transition_to(self.StateMachine.SCANNING)
            else:
                self.transition_to(self.StateMachine.SCANNING)

        # FINDGAP
        elif self.state == self.StateMachine.FINDGAP:
            if self.current_distance and self.current_distance > 0.5:
                self.gap_command_issued = False 
                self.transition_to(self.StateMachine.TRACKING)
            else:
                if not self.gap_command_issued:
                    self.get_logger().info("Analyzing semantic occupancy grid to select evasion heading...")
                    self.gap_command_issued = True
                    self.transition_to(self.StateMachine.TRACKING)

        # Calling
        elif self.state == self.StateMachine.CALLING:
            self.motor.set_wheels_speed(0.0, 0.0)
            self.movement_busy = True

    def destroy_node(self):
        self.motor.set_wheels_speed(0.0, 0.0)
        self.motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlgorithmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()