#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int64, Float32MultiArray, Float32
import math

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        
        self.motor = DaguWheelsDriver()

        # --- ROTATION PD & TRACTION TUNING PARAMETERS ---
        self.MAX_ROT_PWR = 0.32    # Capped slightly lower to preserve traction on slippery floors
        self.MIN_ROT_PWR = 0.15    # Minimum power required to overcome basic mechanical resistance
        
        # PD Controller Gains
        self.KP_ROT = 0.55         # Proportional Gain (determines rotational push)
        self.KD_ROT = 0.04         # Derivative Gain (Damping term - eliminates target oscillations)
        
        # Tolerances
        self.ROT_TOLERANCE = math.radians(2.0)  # Precision success window (2.0 degrees)
        self.SETTLE_TIME = 0.30                 # Must remain stable inside tolerance for 300ms
        self.INTER_TURN_DELAY = 0.1            # 1s electrical and physical recovery pause between chunks

        # Robot Physical Constants
        self.wheel_radius = 0.0325
        self.axle_width = 0.185
        self.encoder_resolution = 140.0
        self.RIGHT_TRIM = 1.105
        
        # Linear Driving Constants
        self.BASE_PWR = 0.45
        self.MIN_STEER_PWR = 0.25
        self.MAX_PWR = 0.6
        self.KP_STRAIGHT = 2.0
        self.RAMP_DISTANCE = 0.15

        # State Variables
        self.state = "IDLE"
        self.target_distance = 0.0
        self.target_degrees = 0.0
        self.target_yaw = 0.0
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.current_yaw = 0.0
        self.yaw_history = []
        self.last_wheel_state = [0, 0]
        
        self.t1_start = 0
        self.t2_start = 0
        self.locked_yaw = 0.0
        self.stop_counter = 0
        self.settle_start_time = None
        self.pause_start_time = None

        # Tracking variables for Multi-step Rotation, PD, and Traction Control
        self.rotation_queue = []            # Holds fragmented sub-turn targets
        self.last_yaw_error = 0.0
        self.rot_pwr_cmd = 0.0

        # IMU Watchdog
        self.last_yaw_time = self.get_clock().now()
        self.imu_timeout_seconds = 0.15

        # Comms
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        self.create_subscription(Float32MultiArray, '/cmd_movement', self.instruction_cb, 10)
        self.create_subscription(Int64, '/encoders/left_ticks', self.left_tick_cb, 10)
        self.create_subscription(Int64, '/encoders/right_ticks', self.right_tick_cb, 10)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_cb, 10)

        # Control Loop (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Unified Smooth Movement & Sequential Turning Node Active.")

    def left_tick_cb(self, msg): self.left_ticks = msg.data
    def right_tick_cb(self, msg): self.right_ticks = msg.data

    def yaw_cb(self, msg): 
        self.last_yaw_time = self.get_clock().now()
        self.yaw_history.append(msg.data)
        if len(self.yaw_history) > 3: 
            self.yaw_history.pop(0)
        sin_sum = sum(math.sin(y) for y in self.yaw_history)
        cos_sum = sum(math.cos(y) for y in self.yaw_history)
        self.current_yaw = math.atan2(sin_sum, cos_sum)

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def instruction_cb(self, msg):
        if self.state != "IDLE" or len(msg.data) < 2: return
        dist, angle = msg.data[0], msg.data[1]
        if dist != 0.0:
            self.target_distance = dist
            self.state = "START_LIN"
        elif angle != 0.0:
            # --- TURN DECOMPOSITION LOGIC ---
            self.rotation_queue = []
            remaining_angle = angle
            
            # Divide large angles into 90-degree chunks
            while abs(remaining_angle) > 90.0:
                sign = 1.0 if remaining_angle > 0 else -1.0
                self.rotation_queue.append(sign * 90.0)
                remaining_angle -= sign * 90.0
            
            # Append any leftover angle remainder
            if remaining_angle != 0.0:
                self.rotation_queue.append(remaining_angle)
            
            # Extract and load the initial chunk target
            if self.rotation_queue:
                self.target_degrees = self.rotation_queue.pop(0)
                self.state = "START_ROT"

    def control_loop(self):
        if self.state == "IDLE": return

        elif self.state == "START_LIN":
            self.t1_start, self.t2_start = self.left_ticks, self.right_ticks
            self.locked_yaw = self.current_yaw
            self.state = "LIN"

        elif self.state == "LIN":
            direction = -1 if self.target_distance > 0 else 1
            self.set_wheel_state(direction, direction)
            cur_l = abs(self.left_ticks - self.t1_start)
            cur_r = abs(self.right_ticks - self.t2_start)
            dist_traveled = ((cur_l + cur_r) / 2.0 / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            remaining = abs(self.target_distance) - dist_traveled

            if dist_traveled >= abs(self.target_distance):
                self.state = "STOPPING"
                return

            ramp_up_factor = min(1.0, dist_traveled / self.RAMP_DISTANCE)
            ramp_down_factor = min(1.0, remaining / self.RAMP_DISTANCE)
            current_base = self.MIN_STEER_PWR + (self.BASE_PWR - self.MIN_STEER_PWR) * min(ramp_up_factor, ramp_down_factor)

            now = self.get_clock().now()
            yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9
            
            # Calculate the raw heading error
            yaw_diff = 0.0 if yaw_age > self.imu_timeout_seconds else self.normalize_angle(self.current_yaw - self.locked_yaw)

            # Inverted direction corrections matching your swapped front/back setup
            if direction == -1: # Target Forwards (Physical Reverse)
                l_pwr = current_base - (yaw_diff * self.KP_STRAIGHT)
                r_pwr = current_base + (yaw_diff * self.KP_STRAIGHT)
            else: # Target Backwards (Physical Forward)
                l_pwr = current_base + (yaw_diff * self.KP_STRAIGHT)
                r_pwr = current_base - (yaw_diff * self.KP_STRAIGHT)

            # Clamp powers safely and apply the right-side trim scaling factor
            l_pwr = max(min(self.MAX_PWR, l_pwr), self.MIN_STEER_PWR)
            r_pwr = max(min(self.MAX_PWR, r_pwr * self.RIGHT_TRIM), self.MIN_STEER_PWR * self.RIGHT_TRIM)
            
            self.motor.set_wheels_speed(l_pwr * direction, r_pwr * direction)

        elif self.state == "START_ROT":
            self.get_logger().info(f"Executing turn segment: {self.target_degrees}° (Queue depth left: {len(self.rotation_queue)})")
            target_rad = -math.radians(self.target_degrees)
            self.target_yaw = self.normalize_angle(self.current_yaw + target_rad)
            self.settle_start_time = None
            
            # Reset tracking baselines cleanly for this chunk
            self.last_yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            self.rot_pwr_cmd = self.MIN_ROT_PWR 
            self.state = "ROT"

        elif self.state == "ROT":
            now = self.get_clock().now()
            yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9

            if yaw_age > self.imu_timeout_seconds:
                self.get_logger().warn("IMU timeout during rotation.")
                self.state = "STOPPING"
                return

            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            abs_error = abs(yaw_error)

            # --- SUCCESS REGION HOOK ---
            if abs_error <= self.ROT_TOLERANCE:
                self.motor.set_wheels_speed(0.0, 0.0)
                self.set_wheel_state(0, 0)

                if self.settle_start_time is None:
                    self.settle_start_time = now
                elif (now - self.settle_start_time).nanoseconds * 1e-9 >= self.SETTLE_TIME:
                    # Segment finished cleanly. Check if there are more sub-turns queued up
                    if len(self.rotation_queue) > 0:
                        self.get_logger().info("Segment complete. Entering recovery pause to stabilize IMU rails...")
                        self.pause_start_time = now
                        self.state = "ROT_PAUSE"
                    else:
                        # Full sequence finished. Proceed to global finish notification
                        self.get_logger().info(f"Full Sequence Complete. Final Error={math.degrees(yaw_error):.2f}°")
                        self.state = "STOPPING"
                return

            self.settle_start_time = None

            # --- DERIVATIVE (DAMPING) CALCULATION ---
            dt = 0.02
            derivative = (yaw_error - self.last_yaw_error) / dt
            self.last_yaw_error = yaw_error

            # --- PD CONTROL SIGNAL GENERATION ---
            pd_signal = (self.KP_ROT * yaw_error) + (self.KD_ROT * derivative)
            
            target_power = abs(pd_signal)
            target_power = max(self.MIN_ROT_PWR, min(self.MAX_ROT_PWR, target_power))

            # --- TRACTION CONTROL ACCELERATION RAMP ---
            if self.rot_pwr_cmd < target_power:
                self.rot_pwr_cmd = min(target_power, self.rot_pwr_cmd + 0.012)
            else:
                self.rot_pwr_cmd = max(target_power, self.rot_pwr_cmd - 0.025)

            power = self.rot_pwr_cmd

            # --- DIRECTION DETERMINATION AND 180° SAFEGUARD ---
            if abs_error > math.radians(172.0):
                left_dir, right_dir = (1, -1) if self.target_degrees > 0 else (-1, 1)
            else:
                left_dir, right_dir = (-1, 1) if pd_signal > 0 else (1, -1)

            self.set_wheel_state(left_dir, right_dir)
            self.motor.set_wheels_speed(
                power * left_dir,
                power * self.RIGHT_TRIM * right_dir
            )

        elif self.state == "ROT_PAUSE":
            # Explicitly enforce 0V on motors to drop back-EMF spikes and settle the chassis physically
            self.motor.set_wheels_speed(0.0, 0.0)
            self.set_wheel_state(0, 0)
            
            now = self.get_clock().now()
            elapsed_pause = (now - self.pause_start_time).nanoseconds * 1e-9
            
            if elapsed_pause >= self.INTER_TURN_DELAY:
                if len(self.rotation_queue) > 0:
                    # Pop next segment and initiate state shift
                    self.target_degrees = self.rotation_queue.pop(0)
                    self.state = "START_ROT"
                else:
                    self.state = "STOPPING"

        elif self.state == "STOPPING":
            self.motor.set_wheels_speed(0.0, 0.0)
            self.set_wheel_state(0, 0)
            self.stop_counter += 1
            if self.stop_counter >= 10:
                self.stop_counter = 0
                self.status_pub.publish(Bool(data=True)) # Published when the entire sequence completes
                self.state = "IDLE"

    def destroy_node(self):
        self.motor.set_wheels_speed(0.0, 0.0)
        self.motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()