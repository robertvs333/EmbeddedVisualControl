#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import math
import time
from enum import Enum, auto
import json

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class AlgorithmNode(Node):

    class StateMachine(Enum):
        INIT = auto()
        SEARCHING = auto()
        SCANNING_ALIGN = auto()  
        SCANNING_CAPTURE = auto()
        FINDGAP = auto()
        TRACKING = auto()
        CALLING = auto()

    def __init__(self):
        super().__init__('algorithm_node')
        
        # Hardware setup
        self.motor = DaguWheelsDriver()

        # --- Internal State Variables ---
        self.state = self.StateMachine.INIT
        self.current_distance = 1.25  
        self.current_yaw = 0.0
        self.movement_busy = False    

        # Vision tracking state trackers
        self.latest_tracking_angle = 0.0
        self.target_detected_in_frame = False
        self.scan_triggered = False
        self.centering_start_time = None
        self.CENTERING_TIMEOUT = 5.0
        self.ALIGNMENT_TOLERANCE_RAD = math.radians(4.0) # Stop tracking when within 4 degrees of center

        # FINDGAP room scanning state trackers
        self.gap_search_start_time = None
        self.best_gap_angle = None
        self.max_gap_distance = 0.0

        # --- Publishers ---
        # The single point of control for robot chassis movement
        self.motion_pub = self.create_publisher(Float32MultiArray, '/cmd_movement', 10)
        # Authorizes sensor_fusion.py to look for targets
        self.scan_trigger_pub = self.create_publisher(String, '/detection/trigger', 10)

        # --- Subscribers ---
        # 1. ToF Distance data (Matches sensor data profile from ToF_sensor.py)
        self.create_subscription(Range, '/tof/distance', self.tof_callback, qos_profile=qos_profile_sensor_data)
        
        # 2. IMU Relative orientation yaw
        self.create_subscription(Float32, '/imu/yaw', self.yaw_callback, 10)
        
        # 3. Movement Node feedback hook
        self.create_subscription(Bool, '/movement_finished', self.movement_finished_callback, 10)
        
        # 4. Continuous live tracking angle stream from sensor_fusion
        self.create_subscription(Float32, '/detection/tracking_angle', self.tracking_angle_callback, 10)
        
        # 5. Final evaluated 10-sample evaluation payload string
        self.create_subscription(String, '/detection/final_result', self.vision_result_callback, 10)

        # --- 20Hz Master Synchronous Loop Clock ---
        self.control_timer = self.create_timer(0.05, self.movement_loop)
        self.get_logger().info("Master Centralized Strategy Node Online.")

    # --- Subscriber Callbacks ---

    def tof_callback(self, msg):
        self.current_distance = msg.range

    def yaw_callback(self, msg):
        self.current_yaw = msg.data

    def movement_finished_callback(self, msg):
        if msg.data is True:
            self.movement_busy = False

    def tracking_angle_callback(self, msg):
        """Receives live offset computations from sensor_fusion stream continuously."""
        self.latest_tracking_angle = msg.data
        # If angle is exactly 0.0 or defaults, it implies nothing is visible in the frame
        if msg.data == 0.0:
            self.target_detected_in_frame = False
        else:
            self.target_detected_in_frame = True

    def vision_result_callback(self, msg):
        """Asynchronously fires when SCANNING_CAPTURE finishes counting its samples."""
        try:
            clean_json = msg.data.replace("'", '"')
            payload = json.loads(clean_json)
            
            target_type = payload.get('_type', 'none')
            confidence = payload.get('confidence', 0.0)
            
            self.get_logger().info(f"Capture results verified: {target_type} ({confidence*100:.1f}%)")

            if self.state == self.StateMachine.SCANNING_CAPTURE:
                self.scan_triggered = False
                self.movement_busy = False

                if target_type == "face":
                    self.transition_to(self.StateMachine.CALLING)
                elif target_type == "object":
                    self.transition_to(self.StateMachine.FINDGAP)
                else:
                    self.transition_to(self.StateMachine.SEARCHING)
        except Exception as e:
            self.get_logger().error(f"Error reading result dictionary string: {e}")
            self.transition_to(self.StateMachine.SEARCHING)

    # --- Command Utilities ---

    def transition_to(self, next_state):
        self.get_logger().info(f"State Transition: {self.state.name} -> {next_state.name}")
        self.state = next_state

    def send_drive_command(self, distance, angle_deg):
        msg = Float32MultiArray()
        msg.data = [float(distance), float(angle_deg)]
        self.motion_pub.publish(msg)
        self.movement_busy = True

    # --- Master Synchronous Control Loop ---

    def movement_loop(self):
        # Guard clause: Wait if the movement_node is actively driving or executing an absolute chunk turn
        if self.movement_busy:
            return

        # INIT STATE: Validate startup clearances
        if self.state == self.StateMachine.INIT:
            if self.current_distance >= 0.3:
                self.transition_to(self.StateMachine.SEARCHING)
            else:
                self.get_logger().warn("Obstacle blocking start. Moving back...")
                self.send_drive_command(-0.15, 0.0)

        # SEARCHING STATE: Forward scanning path progression
        elif self.state == self.StateMachine.SEARCHING:
            if self.current_distance >= 0.3:
                self.send_drive_command(0.2, 0.0)
            else:
                self.get_logger().info("Obstacle boundary hit. Entering alignment mode.")
                self.centering_start_time = time.time()
                self.transition_to(self.StateMachine.SCANNING_ALIGN)

        # SCANNING_ALIGN STATE: Algorithm-driven target tracking loops
        elif self.state == self.StateMachine.SCANNING_ALIGN:
            # Check for safety timeout
            if time.time() - self.centering_start_time > self.CENTERING_TIMEOUT:
                self.get_logger().warn("Centering phase timed out. Proceeding to find gap.")
                self.transition_to(self.StateMachine.FINDGAP)
                return

            if self.target_detected_in_frame:
                # If offset angle falls inside our acceptable margin, stop turning and capture data
                if abs(self.latest_tracking_angle) <= self.ALIGNMENT_TOLERANCE_RAD:
                    self.get_logger().info("Chassis centered with target. Commencing data snapshot capture.")
                    self.transition_to(self.StateMachine.SCANNING_CAPTURE)
                else:
                    # Convert rad error to degrees for our standard movement_node API
                    turn_step_deg = math.degrees(self.latest_tracking_angle)
                    # Feed proportional corrections safely via movement_node execution blocks
                    self.send_drive_command(0.0, turn_step_deg)
            else:
                # If we hit an obstacle but sensor_fusion sees absolutely nothing, skip to open path searches
                self.get_logger().info("No targets visible in current view. Checking for environmental gaps.")
                self.transition_to(self.StateMachine.FINDGAP)

        # SCANNING_CAPTURE STATE: Safe static image collection zone
        elif self.state == self.StateMachine.SCANNING_CAPTURE:
            if not self.scan_triggered:
                trigger_msg = String()
                trigger_msg.data = "START_SCAN"
                self.scan_trigger_pub.publish(trigger_msg)
                self.scan_triggered = True
                self.movement_busy = True # Force structural wait until vision_result_callback unlocks it

        # FINDGAP STATE: Sweep room boundaries to isolate clear corridors
        elif self.state == self.StateMachine.FINDGAP:
            if self.gap_search_start_time is None:
                self.gap_search_start_time = time.time()
                self.best_gap_angle = None
                self.max_gap_distance = 0.0
                self.get_logger().info("Starting full 360 rotation environmental gap sweep...")
                self.send_drive_command(0.0, 360.0)
                return

            if self.current_distance > self.max_gap_distance:
                self.max_gap_distance = self.current_distance
                self.best_gap_angle = self.current_yaw

            if not self.movement_busy or (time.time() - self.gap_search_start_time > 12.0):
                self.gap_search_start_time = None
                if self.max_gap_distance >= 0.6 and self.best_gap_angle is not None:
                    self.transition_to(self.StateMachine.TRACKING)
                else:
                    self.get_logger().error("No viable paths found. Resetting state machine.")
                    self.transition_to(self.StateMachine.INIT)

        # 6. TRACKING STATE: Aim chassis down the selected escape corridor
        elif self.state == self.StateMachine.TRACKING:
            yaw_error_rad = self.best_gap_angle - self.current_yaw
            yaw_error_rad = (yaw_error_rad + math.pi) % (2.0 * math.pi) - math.pi
            yaw_error_deg = math.degrees(yaw_error_rad)

            self.get_logger().info(f"Re-orienting chassis into open corridor path: {yaw_error_deg:.1f}°")
            self.send_drive_command(0.0, yaw_error_deg)
            self.transition_to(self.StateMachine.SEARCHING)

        # 7. CALLING STATE: Goal termination
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