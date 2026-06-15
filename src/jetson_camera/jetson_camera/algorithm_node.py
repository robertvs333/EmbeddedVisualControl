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
        
        # Structural parameters
        self.scan_total_angle = 0.0
        self.scan_target_max = 90.0   
        self.scan_step_deg = 15.0     
        
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
        self.map_pub = self.create_publisher(String, '/vision/detection_result', qos_profile=mapping_qos)
        matching_latching_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(Range, '/tof/distance', self.tof_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_callback, 10)
        self.create_subscription(Bool, '/movement_finished', self.movement_finished_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/tracking_angles', self.tracking_angle_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/face_tracking_angles', self.face_tracking_angle_callback, 10)
        self.create_subscription(OccupancyGrid, '/mapping/semantic_grid', self.map_callback, 10)
        self.mapping_status_sub = self.create_subscription(Bool,'/mapping/active', self.mapping_status_callback, qos_profile=matching_latching_qos)
        self.final_result_sub = self.create_subscription( Bool, '/detection/final_result', self.final_result_callback, 10)
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
            
            # Check if scan is complete and objects were actually found
            if payload.get("status") == "scan_complete" and payload.get("detected_object_count", 0) > 0:
                primary_obj = payload["objects"][0]
                
                # Coarse Object Alignment Check
                if not self.has_aligned:
                    # Capture relative object angle from frame center (convert to degrees if sent in radians)
                    self.latest_tracking_angle = primary_obj["angle"]
                    self.target_detected_in_frame = True
                    self.get_logger().info(f"Object detected at offset {self.latest_tracking_angle:.2f}. Running alignment turn...")
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return
                
                # Log object data to mapping node once aligned
                self.process_and_publish_map_entry(target_type="object", confidence=1.0)
                
                # Advance to Tier 2 (Face Recognition)
                self.object_scan_done = True
                self.scan_triggered = False  
                
            else:
                # --- BREAKS THE PING-PONG LOOP HERE ---
                self.get_logger().info("No objects detected at this angle step.")
                self.advance_sweep_sequence()
                
        except Exception as e:
            self.get_logger().error(f"Error parsing object payload: {e}")
            # Ensure a parser failure doesn't deadlock the robot state machine forever
            self.advance_sweep_sequence()

    def advance_sweep_sequence(self):
        """Increments the physical search angle, commands motor rotation, and updates state."""
        # Reset tracking status flags for this individual frame window step
        self.scan_triggered = False
        self.object_scan_done = False
        self.target_detected_in_frame = False
        
        # Accumulate our search boundary step (e.g., 15.0 degrees)
        self.scan_total_angle += self.scan_step_deg
        
        if self.scan_total_angle < self.scan_target_max:
            self.get_logger().info(f"Advancing sweep window. Turning next step: +{self.scan_step_deg}° (Total: {self.scan_total_angle}°/{self.scan_target_max}°)")
            self.send_drive_command(0.0, self.scan_step_deg) 
            self.movement_busy = True  
            self.transition_to(self.StateMachine.SEARCHING)
        else:
            self.get_logger().warn("Full 90° workspace area sweep completed with no targets. Resetting boundary bounds.")
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
            allowed_identities = ["trump", "Markie", "Musk", "Geert"]
            if identity in allowed_identities:
                self.get_logger().info(f"SUCCESS: Face verified! Identity -> [{identity}].")
                self.face_found_globally = True
                self.process_and_publish_map_entry(target_type=f"face_{identity}", confidence=1.0)
                
                if self.face_detected_in_frame and abs(self.latest_face_tracking_angle) > 0.05 and not self.face_scan_done:
                    self.get_logger().info(f"Face identified [{identity}], executing final precision centering adjustment.")
                    
                    self.latest_tracking_angle = self.latest_face_tracking_angle
                    self.target_detected_in_frame = True
                    self.face_scan_done = True 
                    
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return

                self.face_scan_done = True
                self.advance_sweep_sequence()
                self.transition_to(self.StateMachine.CALLING)
            
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
                
            obj_type = target_type if target_type is not None else object_payload.get('type', 'unidentified')
            confidence = object_payload.get('confidence', kwargs.get('confidence', 1.0))
            distance = object_payload.get('distance', kwargs.get('distance', 0.3))
            angle = object_payload.get('angle', kwargs.get('angle', 0.0))

            payload_dict = {
                "type": str(obj_type),
                "distance_m": float(distance),
                "angle_deg": float(angle),
                "confidence": float(confidence)
            }

            msg = String()
            msg.data = json.dumps(payload_dict)
            
            if hasattr(self, 'map_pub') and self.map_pub is not None:
                self.map_pub.publish(msg)
                self.get_logger().info(f"Published to grid mapper: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error inside process_and_publish_map_entry: {e}")


    def final_result_callback(self, msg):
        """Callback to receive the finish signal from vision nodes."""
        if msg.data:
            self.get_logger().info("Received final vision detection signal. Breaking out of SCANNING_CAPTURE.")
            self.detection_received = True

            if self.current_state == "SCANNING_CAPTURE":
                self.current_state = "SCANNING"

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
                self.sweep_start_yaw = None # Reset baseline
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
            self.movement_busy = True
            
            self.get_logger().info(f"Sweeping: Advancing step turn of {self.scan_step_deg}°")
            self.send_drive_command(0.0, self.scan_step_deg)
            self.scan_total_angle += self.scan_step_deg
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
                self.get_logger().info("Face captured")
                self.transition_to(self.StateMachine.CALLING)


    # SCANNING ALLIGN: 
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
                self.transition_to(self.StateMachine.SCANNING_CAPTURE)

            else:
                self.get_logger().warn("Alignment target lost frame reference. Resuming search loop.")
                self.clean_alignment_and_resume()


    # TRACKING:
        elif self.state == self.StateMachine.TRACKING:
            self.get_logger().info("TRACKING: Evaluating map to find clear path...")
            
            if not self.gap_command_issued:
                target_distance, target_heading = self.find_next_target()

                self.send_drive_command(target_distance, target_heading)
    
            elif self.gap_command_issued:
                escape_heading = self.find_clear_tracking_angle(target_distance_m=0.4)
                self.best_gap_angle = escape_heading
                self.send_drive_command(0.35, escape_heading)
            
            self.scan_total_angle = 0.0
            self.scan_target_max = 180.0 
            self.transition_to(self.StateMachine.SEARCHING)


    # SEARCHING 
        elif self.state == self.StateMachine.SEARCHING:
            if abs(self.scan_total_angle) >= self.scan_target_max:
                # Gatekeeper: Must check that the face has been detected before calling mission end!
                if self.face_found_globally:
                    self.get_logger().info("Wide area SEARCHING complete and face verified. Moving to CALLING.")
                    self.transition_to(self.StateMachine.CALLING)
                else:
                    self.get_logger().warn("Wide sweep completed but face reference is missing. Re-running wide area sweep loop.")
                    self.scan_total_angle = 0.0 # Reset sweep accumulator to stay active
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