#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import math
import time
from enum import Enum, auto
import json

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

# IMPORT YOUR UTILS MIXIN FROM THE SEPARATE FILE HERE:
from algorithm_util import Algorithm_utils

# ADD Algorithm_utils RIGHT BESIDE Node IN THE HERITAGE LINE:
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
        # Initialize the underlying ROS 2 Node first
        super().__init__('algorithm_node')
        
        self.required_nodes = [
            'tof_node',
            'imu_node',
            'Face_detection_recognition',
            'movement_node',
            'object_detection',
            'plan_route',
        ]
        
        self.is_robot_ready = False
        
        # Block startup until the full network topology is online
        # (This successfully maps directly into your imported file!)
        self.wait_for_system_mesh()

        # Hardware setup
        self.motor = DaguWheelsDriver()

        # --- Internal State Variables ---
        self.face_found_globally = False
        self.state = self.StateMachine.BOOTING
        self.current_distance = None  
        self.current_yaw = 0.0
        self.movement_busy = False    
        self.current_map = None
        
        # Tracking angle registries
        self.latest_tracking_angle = 0.0
        self.target_detected_in_frame = False
        self.latest_face_tracking_angle = 0.0
        self.face_detected_in_frame = False
        
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
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_movement', 10)
        self.scan_trigger_pub = self.create_publisher(String, '/detection/trigger', 10)
        self.object_trigger_pub = self.create_publisher(String, '/detection/trigger', 10)
        self.face_trigger_pub = self.create_publisher(String, '/detection/face_trigger', 10)
        self.scan_result = self.create_publisher(String, '/detection/map_result', 10)

        # Notice how these bindings match perfectly with methods inside your utility class:
        self.create_subscription(Range, '/tof/distance', self.tof_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_callback, 10)
        self.create_subscription(Bool, '/movement_finished', self.movement_finished_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/tracking_angles', self.tracking_angle_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/face_tracking_angles', self.face_tracking_angle_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Main Processing Execution Synchronizers
        self.create_subscription(String, '/detection/final_result', self.object_result_callback, 10)
        self.create_subscription(String, '/detection/face_result', self.face_result_callback, 10)

        # Heartbeat clock processing loop
        self.timer = self.create_timer(0.05, self.movement_loop)

       



    def object_result_callback(self, msg):
        """Processes Tier 1 outputs from MultiObjectDetectionNode."""
        if self.state != self.StateMachine.SCANNING_CAPTURE or self.object_scan_done:
            return
            
        try:
            payload = json.loads(msg.data.replace("'", '"'))
            if payload.get("status") == "scan_complete" and payload.get("detected_object_count", 0) > 0:
                primary_obj = payload["objects"][0]
                
                # Coarse Object Alignment Check
                if not self.has_aligned:
                    # Capture relative object angle from frame center (in radians)
                    self.latest_tracking_angle = primary_obj["angle"]
                    self.target_detected_in_frame = True
                    self.get_logger().info("Object detected. Running coarse alignment turn...")
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return
                
                # Log object data to mapping node once aligned
                self.process_and_publish_map_entry(target_type="object", confidence=1.0)
                
                # Advance to Tier 2 (Face Recognition)
                self.object_scan_done = True
                self.scan_triggered = False  
            else:
                self.get_logger().info("No objects detected at this angle step.")
                self.resume_search_sequence()
                
        except Exception as e:
            self.get_logger().error(f"Error parsing object payload: {e}")
            self.resume_search_sequence()

    def face_result_callback(self, msg):
        """Processes final summary outputs from FaceRecognitionNode."""
        if self.state != self.StateMachine.SCANNING_CAPTURE or not self.object_scan_done:
            return

        try:
            payload = json.loads(msg.data.replace("'", '"'))
            identity = payload.get("identity", "Unknown")
            
            if identity != "Unknown":
                # Check for a remaining off-center offset using our new tracking callback variables
                if self.face_detected_in_frame and abs(self.latest_face_tracking_angle) > 0.05 and not self.face_scan_done:
                    self.get_logger().info(f"Face identified [{identity}], but needs fine alignment adjustment.")
                    
                    # Pass the localized tracker array variable into the primary state register
                    self.latest_tracking_angle = self.latest_face_tracking_angle
                    self.target_detected_in_frame = True
                    
                    self.face_scan_done = True 
                    self.transition_to(self.StateMachine.SCANNING_ALIGN)
                    return

                # Face is aligned: Send result to map node
                self.get_logger().info(f"SUCCESS: Face verified! Identity -> [{identity}].")
                self.process_and_publish_map_entry(target_type=f"face_{identity}", confidence=1.0)
                self.face_found_globally = True
            else:
                self.get_logger().info("Face scan complete: No known faces identified.")
            
            self.face_scan_done = True
            self.clean_alignment_and_resume()
            
        except Exception as e:
            self.get_logger().error(f"Error parsing face summary payload: {e}")
            self.clean_alignment_and_resume()
    
    def movement_loop(self):
        if self.check_for_motor_stall():
            return
        # Guard clause: Wait if the movement_node is actively driving 
        if self.movement_busy:
            return

    # BOOTING state
        if self.state == self.StateMachine.BOOTING:
            if self.wait_for_system_mesh():
                if self.verify_active_publishers():
                    self.is_robot_ready = True
                    self.transition_to(self.StateMachine.INIT)  

    # INIT STATE: Validate startup clearances
        elif self.state == self.StateMachine.INIT:
            self.scan_total_angle = 0.0
            self.scan_target_max = 90.0   
            self.scan_step_deg = 15.0     
            self.has_aligned = False
            self.scan_results = []
            self.get_logger().info("System Initialized. Entering SEARCHING_INIT.")
            self.transition_to(self.StateMachine.SEARCHING_INIT)


    # SEARCH INIT STATE: 
        elif self.state == self.StateMachine.SEARCHING_INIT:
            if abs(self.scan_total_angle) >= self.scan_target_max:
                # Gatekeeper: Only move to TRACKING if a face has been confirmed!
                if self.face_found_globally:
                    self.get_logger().info("Initial sweep complete and face found. Moving to TRACKING.")
                    self.transition_to(self.StateMachine.TRACKING)
                else:
                    self.get_logger().warn("Initial sweep completed but NO face detected yet. Repeating initial sweep window.")
                    self.scan_total_angle = 0.0 # Reset sweep accumulator to loop again
                    self.transition_to(self.StateMachine.SCANNING)
            else:
                self.transition_to(self.StateMachine.SCANNING)


    # SCANNING: 
        elif self.state == self.StateMachine.SCANNING:
            self.has_aligned = False  
            self.object_scan_done = False
            self.face_scan_done = False
            self.scan_triggered = False
            
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


    # SCANNING ALLIGN: 
        elif self.state == self.StateMachine.SCANNING_ALIGN:
            if self.target_detected_in_frame:
                step_offset = math.degrees(self.latest_tracking_angle)
                self.get_logger().info(f"Aligning camera axis. Turn offset: {step_offset:.2f}°")
                
                self.has_aligned = True
                self.alignment_offset_deg += step_offset 
                
                # Turn to center the target
                self.send_drive_command(0.0, step_offset)
                
                # Clear frame memory flags before executing the centered capture
                self.scan_triggered = False 
                self.target_detected_in_frame = False
                self.face_detected_in_frame = False
                
                self.transition_to(self.StateMachine.SCANNING_CAPTURE)
            else:
                self.get_logger().warn("Alignment target lost frame reference. Resuming search loop.")
                self.clean_alignment_and_resume()


    # TRACKING:
        elif self.state == self.StateMachine.TRACKING:
            self.get_logger().info("TRACKING: Evaluating map to find clear space...")
            
            # Search the occupancy grid for a clear direction to move
            clear_relative_angle = self.find_clear_tracking_angle(target_distance_m=0.25)
            
            # Turn toward the clear open space and drive forward 0.25 meters
            self.send_drive_command(0.25, clear_relative_angle)
            
            # Configure loop parameters for the permanent wide-angle SEARCHING state
            self.scan_total_angle = 0.0
            self.scan_target_max = 180.0  # Expanded search angle for the second loop
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
            # Re-verify distance and try to recover from the collision stall
            if self.current_distance and self.current_distance > 0.5:
                self.transition_to(self.StateMachine.TRACKING)
            else:
                # Fall back to your default full 360 gap recovery sweep
                self.best_gap_angle = None
                self.max_gap_distance = 0.0
                self.send_drive_command(0.0, 360.0)

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