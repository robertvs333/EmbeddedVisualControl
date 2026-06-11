#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Float32MultiArray, OccupancyGrid
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import math
import time
from enum import Enum, auto
import json

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class AlgorithmNode(Node):

    class StateMachine(Enum):
        BOOTING = auto()
        NETWORK_READY = auto()
        INIT = auto()
        SEARCHING = auto()
        SCANNING_ALIGN = auto()  
        SCANNING_CAPTURE = auto()
        FINDGAP = auto()
        TRACKING = auto()
        CALLING = auto()

    def __init__(self):
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
        self.wait_for_system_mesh()

        # Hardware setup
        self.motor = DaguWheelsDriver()

        # --- Internal State Variables ---
        self.state = self.StateMachine.BOOTING
        self.current_distance = None  
        self.current_yaw = 0.0
        self.movement_busy = False    

        # Vision tracking state trackers
        self.latest_tracking_angle = 0.0
        self.target_detected_in_frame = False
        self.scan_triggered = False
        self.centering_start_time = None
        self.CENTERING_TIMEOUT = 5.0
        self.ALIGNMENT_TOLERANCE_RAD = math.radians(4.0) 

        # FINDGAP room scanning state trackers
        self.gap_search_start_time = None
        self.best_gap_angle = None
        self.max_gap_distance = 0.0

        # --- Publishers ---
        self.motion_pub = self.create_publisher(Float32MultiArray, '/cmd_movement', 10)
        self.scan_trigger_pub = self.create_publisher(String, '/detection/trigger', 10)
        self.scan_result = self.create_publisher(String, '/detection/map_result', 10)

        # --- Subscribers ---
        self.create_subscription(Range, '/tof/distance', self.tof_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_callback, 10)
        self.create_subscription(Bool, '/movement_finished', self.movement_finished_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/tracking_angle', self.tracking_angle_callback, 10)
        self.create_subscription(String, '/detection/final_result', self.vision_result_callback, 10)
        self.create_subscription(OccupancyGrid, '/mapping/semantic_grid',10)
        
        # --- 20Hz Master Synchronous Loop Clock ---
        self.control_timer = self.create_timer(0.05, self.movement_loop)


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

            ## aanpassen voor twee detectie nodes
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


    # def map_callback(self, msg):
    #     self.obstacle

    # --- Command Utilities ---

    def transition_to(self, next_state):
        self.get_logger().info(f"State Transition: {self.state.name} -> {next_state.name}")
        self.state = next_state

    def send_drive_command(self, distance, angle_deg):
        msg = Float32MultiArray()
        msg.data = [float(distance), float(angle_deg)]
        self.motion_pub.publish(msg)
        self.movement_busy = True

    def movement_loop(self):
        # Guard clause: Wait if the movement_node is actively driving 
        if self.movement_busy:
            return

        # BOOTING state
        if self.state == self.StateMachine.BOOTING and rclpy.ok():
            current_nodes = self.get_node_names()
            missing_nodes = [node for node in self.required_nodes if node not in current_nodes]
            self.get_logger().info("Initializing System Verification...")

            if not missing_nodes:
                self.get_logger().info("Graph Verified: All nodes are active")           
                if self.verify_active_publishers():
                    self.state = self.State_Machine.INIT
            else:
                self.get_logger().warn(
                    f"Startup Blocked. Waiting for missing nodes: {missing_nodes}. "
                )
            time.sleep(1.0)

        # INIT STATE: Validate startup clearances
        if self.state == self.StateMachine.INIT:
            trigger_msg = String()
            trigger_msg.data = "START_SCAN"
            
            # self.transition_to(self.StateMachine.SEARCHING)

            # if self.current_distance >= 0.3:
            #     self.transition_to(self.StateMachine.SEARCHING)
            # else:
            #     self.get_logger().warn("Obstacle blocking start. Moving back...")
            #     self.send_drive_command(-0.15, 0.0)





        # SEARCHING STATE: Forward scanning path progression
        # om de 15 graden draaien, nieuwe scan, check hit, publish hit naar map, en opnieuw tot je de 
            # total_angle = angle
            # turn_angle = 15
            # nieuwe scan --> target angle transition_to SCANNING ALLIGN 
            # scanning allign done --> publish hit naar map (wait 1 sec)
            # number_scan += 1
            # if number_scan >= int(total_angle / turn_angle) 
            #    return

            # init maar hoek van 90 graden checken
                # align is ook rijd naar voren (tof, distance - 0.2) zodat scan 
                # klaar met allign --> terug naar home positie.

                # home positie wordt aan gepast als robot alle stappen van hoeken heeft gehad 
                # algoritme houd bij waar robot is en waar hij vandaan komt
                    # [ home_x, home_y, robot_yaw, ]

            # daarna is search state ongeveer 180 graden of meer, (hoek die hij nog niet gezien heeft)
            # angle increments 15 degrees
            


        # if self.state == self.StateMachine.SEARCHING:
        #     # if self.current_distance is not self.object_distance and self.current_yaw is not self.object_yaw:
        #     #     driving_distance = self.object_distance - self.current_distance
        #     #     driving_yaw = self.object_yaw - self.current_yaw
        #     #     self.send_drive_command(0.0, driving_yaw)
        #     #     if 
        #     #     self.send_drive_command(driving_distance, 0.0)

        #     else:
        #         self.get_logger().info("Obstacle boundary hit. Entering alignment mode.")
        #         self.centering_start_time = time.time()
        #         self.transition_to(self.StateMachine.SCANNING_ALIGN)

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
                    turn_step_deg = math.degrees(self.latest_tracking_angle)
                    self.send_drive_command(0.0, turn_step_deg)
            else:
                self.get_logger().info("No targets visible in current view. Checking for environmental gaps.")
                self.transition_to(self.StateMachine.FINDGAP)

        # SCANNING_CAPTURE STATE: Safe static image collection zone
        elif self.state == self.StateMachine.SCANNING_CAPTURE:
            if not self.scan_triggered:
                trigger_msg = String()
                trigger_msg.data = "START_SCAN"
                self.scan_trigger_pub.publish(trigger_msg)
                self.scan_triggered = True
                self.movement_busy = True 

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

        # TRACKING STATE: Aim chassis down the selected escape corridor
        elif self.state == self.StateMachine.TRACKING:
            yaw_error_rad = self.best_gap_angle - self.current_yaw
            yaw_error_rad = (yaw_error_rad + math.pi) % (2.0 * math.pi) - math.pi
            yaw_error_deg = math.degrees(yaw_error_rad)

            self.get_logger().info(f"Re-orienting chassis into open corridor path: {yaw_error_deg:.1f}°")
            self.send_drive_command(0.0, yaw_error_deg)
            self.transition_to(self.StateMachine.SEARCHING)

<<<<<<< HEAD
            # extra loop om te checken of hij draait/beweegt via imu

        # 7. CALLING STATE: Goal termination
=======
        # CALLING STATE: Goal termination
>>>>>>> d8980cadae3293e6b82541f559f1c276bd1afac9
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