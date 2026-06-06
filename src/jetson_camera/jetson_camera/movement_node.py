#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int64, Float32MultiArray, Float32
import math
import time

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        
        self.motor = DaguWheelsDriver()

        # Robot Constants
        self.wheel_radius = 0.0325
        self.axle_width = 0.185
        self.encoder_resolution = 140.0
        
        # --- CALIBRATION & LIMITS (SLOW & PRECISE) ---
        # Right motor is physically weaker, so it gets 1.105x more command power to match the Left
        self.RIGHT_TRIM = 1.105        
        self.BASE_PWR = 0.5          # Cruise linear speed
        self.MIN_STEER_PWR = 0.3     # Minimum motor torque floor (decel target/ramp start)
        self.MAX_PWR = 0.7           

        # Straight PID heading lock
        self.KP_STRAIGHT = 2.5        
        
        # Linear Ramping Parameters
        self.RAMP_DISTANCE = 0.15     # Distance in meters (15cm) used to ramp up and ramp down

        # State Machine Variables
        self.state = "IDLE"           # IDLE, START_LIN, LIN, START_ROT, ROT, STOPPING
        self.target_distance = 0.0
        self.target_degrees = 0.0
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.current_yaw = 0.0        
        self.yaw_history = []         # Stores recent yaw samples for rolling filter
        self.last_wheel_state = [0, 0]
        
        # Tracking variables
        self.t1_start = 0
        self.t2_start = 0
        self.locked_yaw = 0.0
        self.stop_counter = 0

        # IMU Watchdog variables [NEW]
        self.last_yaw_time = self.get_clock().now()
        self.imu_timeout_seconds = 0.25  # 10 missed frames at 40Hz
        self.imu_warning_logged = False

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        # Subscribers
        self.create_subscription(Float32MultiArray, '/cmd_movement', self.instruction_cb, 10)
        self.create_subscription(Int64, '/encoders/left_ticks', self.left_tick_cb, 10)
        self.create_subscription(Int64, '/encoders/right_ticks', self.right_tick_cb, 10)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_cb, 10)

        # 50Hz Control Loop Timer
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Precision State-Machine Movement Node Active.")

    def left_tick_cb(self, msg): self.left_ticks = msg.data
    def right_tick_cb(self, msg): self.right_ticks = msg.data

    def yaw_cb(self, msg): 
        """Smooths out IMU yaw data using a Circular Rolling Average."""
        # Update watchdog timer [NEW]
        self.last_yaw_time = self.get_clock().now()

        self.yaw_history.append(msg.data)
        if len(self.yaw_history) > 5: # 5-sample moving window (~125ms of data)
            self.yaw_history.pop(0)
        
        # Circular mean to prevent averaging bugs near the -180/180 boundary
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
        if self.state != "IDLE" or len(msg.data) < 2: 
            return
        
        dist, angle = msg.data[0], msg.data[1]
        
        if dist != 0.0:
            self.target_distance = dist
            self.state = "START_LIN"
        elif angle != 0.0:
            self.target_degrees = angle
            self.state = "START_ROT"

    def control_loop(self):
        if self.state == "IDLE":
            return

        elif self.state == "START_LIN":
            self.get_logger().info(f"Starting Linear Move of {self.target_distance}m")
            self.t1_start, self.t2_start = self.left_ticks, self.right_ticks
            self.locked_yaw = self.current_yaw
            self.imu_warning_logged = False
            self.state = "LIN"

        elif self.state == "LIN":
            direction = -1 if self.target_distance > 0 else 1
            self.set_wheel_state(direction, direction)

            # Calculate current distance
            cur_l = abs(self.left_ticks - self.t1_start)
            cur_r = abs(self.right_ticks - self.t2_start)
            dist_traveled = ((cur_l + cur_r) / 2.0 / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            remaining = abs(self.target_distance) - dist_traveled

            if dist_traveled >= abs(self.target_distance):
                self.state = "STOPPING"
                return

            # --- DYNAMIC LINEAR ACCELERATION & DECELERATION PROFILE ---
            # Linearly ramps up over the first 15cm and ramps down over the last 15cm
            ramp_up_factor = min(1.0, dist_traveled / self.RAMP_DISTANCE)
            ramp_down_factor = min(1.0, remaining / self.RAMP_DISTANCE)
            combined_factor = min(ramp_up_factor, ramp_down_factor)
            
            current_base = self.MIN_STEER_PWR + (self.BASE_PWR - self.MIN_STEER_PWR) * combined_factor

            # --- WATCHDOG: CHECK IMU STATUS [NEW] ---
            now = self.get_clock().now()
            yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9

            if yaw_age > self.imu_timeout_seconds:
                # IMU is lagging or disconnected: temporarily disable heading correction
                correction = 0.0
                if not self.imu_warning_logged:
                    self.get_logger().warn("IMU watchdog timeout! Disabling heading lock. Falling back to open-loop trim.")
                    self.imu_warning_logged = True
            else:
                # IMU is healthy: perform heading correction
                self.imu_warning_logged = False
                yaw_error = self.normalize_angle(self.current_yaw - self.locked_yaw)
                correction = yaw_error * self.KP_STRAIGHT * direction

            l_pwr = max(min(self.MAX_PWR, current_base + correction), self.MIN_STEER_PWR)
            r_pwr = max(min(self.MAX_PWR, (current_base - correction) * self.RIGHT_TRIM), self.MIN_STEER_PWR * self.RIGHT_TRIM)

            self.motor.set_wheels_speed(l_pwr * direction, r_pwr * direction)

        elif self.state == "START_ROT":
            self.get_logger().info(f"START_ROT skeleton active for turn of {self.target_degrees}°")
            # --- SKELETON PLACEHOLDER ---
            self.state = "ROT"

        elif self.state == "ROT":
            # --- WATCHDOG: ROTATION SAFETY CHECK [NEW] ---
            # If turning relies on IMU and it drops out, we must stop immediately to avoid infinite rotation.
            now = self.get_clock().now()
            yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9
            if yaw_age > self.imu_timeout_seconds:
                self.get_logger().error("IMU signal lost during rotation! Emergency stop initiated.")
                self.state = "STOPPING"
                return

            # --- SKELETON PLACEHOLDER ---
            self.get_logger().info("ROT skeleton active. Simulating turn completion.", once=True)
            self.state = "STOPPING"

        elif self.state == "STOPPING":
            # Multi-cycle braking sequence
            self.motor.set_wheels_speed(0.0, 0.0)
            self.set_wheel_state(0, 0)
            self.stop_counter += 1
            if self.stop_counter >= 5: 
                self.stop_counter = 0
                
                status_msg = Bool()
                status_msg.data = True
                self.status_pub.publish(status_msg)
                self.get_logger().info("Sequence complete. Stopped.")
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