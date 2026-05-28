#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver
import time
import math
import numpy as np

class PositionController:
    def __init__(self, wheel_radius=0.033, axle_width=0.19, trim=0.0):
        self.motor = DaguWheelsDriver()
        self.wheel_radius = wheel_radius
        self.wheel_base = axle_width
        self.trim = trim
        self.resolution = 140.0
        self.left_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_1)
        self.right_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_2)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.left_gain = 1.0   
        self.right_gain = 1.0  

    def cmd_to_rad_per_s(self, left_cmd, right_cmd):
        return self.left_gain * left_cmd, self.right_gain * right_cmd

    def close(self):
        self.motor.close()


class SmoothSingleWheelCenteringNode(Node):
    def __init__(self):
        super().__init__('smooth_single_wheel_centering_node')
        self.get_logger().info("Initializing Non-Blocking Single-Wheel Centering Node...")

        # Initialize the robot controller configuration
        self.controller = PositionController(trim=0.0)
        
        # 1. Update subscription topic to match the Integrated Fusion Node output
        self.subscription = self.create_subscription(
            Float32,
            '/detection/tracking_angle',
            self.angle_callback,
            10  
        )
        
        # --- 2. Tunable Proportional Parameters ---
        self.kp = 1.3              # Proportional gain: scales right wheel speed based on error size
        self.deadband_rad = 0.05   # Roughly 3 degrees. Stops tracking adjustments once close enough
        self.max_turn_speed = 0.35 # Structural speed cap for your right wheel
        
        # 3. Safety Watchdog Variables
        self.last_msg_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info("Centering node is running in low-power listening mode!")

    def angle_callback(self, msg):
        """Processes target angles asynchronously at 30Hz without blocking the executor thread."""
        self.last_msg_time = time.time()
        target_angle = msg.data

        # Explicit stop signal from laptop or error inside deadband threshold
        if target_angle == 0.0 or abs(target_angle) < self.deadband_rad:
            self.controller.motor.set_wheels_speed(left=0.0, right=0.0)
            return

        # Compute proportional output command
        turn_command = self.kp * target_angle
        
        # Clip command within safe velocity bounds
        turn_command = max(min(turn_command, self.max_turn_speed), -self.max_turn_speed)

        # --- ASYMMETRIC SINGLE-WHEEL DRIVE LOGIC ---
        # Per your accurate hardware setup, the left wheel is strictly anchored at zero
        left_wheel_speed = 0.0
        
        # Sign Mapping Matching Your Chassis Configuration:
        # If target_angle > 0 (Target is on Left), turn_command is positive.
        # Driving the right wheel forward (+) spins the vehicle counter-clockwise (turns left).
        # If target_angle < 0 (Target is on Right), turn_command is negative.
        # Driving the right wheel backward (-) spins the vehicle clockwise (turns right).
        right_wheel_speed = turn_command + (self.controller.trim * turn_command)

        # Apply speeds directly to the motors over I2C instantly
        self.controller.motor.set_wheels_speed(left=left_wheel_speed, right=right_wheel_speed)

    def watchdog_check(self):
        """Safety Guardrail: If the laptop drops its Wi-Fi connection mid-turn, brake immediately."""
        if time.time() - self.last_msg_time > 0.4:
            self.controller.motor.set_wheels_speed(left=0.0, right=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothSingleWheelCenteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Stopping motors...")
    finally:
        node.controller.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()