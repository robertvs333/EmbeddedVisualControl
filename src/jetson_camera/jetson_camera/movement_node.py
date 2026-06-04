#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Float32MultiArray, Float32
import math
import time

import message_filters
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        
        # Hardware setup
        self.motor = DaguWheelsDriver()
        self.left_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_1)
        self.right_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_2)

        # Robot constants
        self.wheel_radius = 0.0325
        self.axle_width = 0.19
        self.encoder_resolution = 147.0
        
        # PID Constants (From PDF)
        self.SAMPLETIME = 0.1 
        self.TARGET = 25       
        self.KP = 0.02
        self.KD = 0.01
        self.KI = 0.005

        self.last_wheel_state = [0, 0]

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        # --- FIXED: UNCOMMENTED SUBSCRIBER ---
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 
            '/cmd_movement', 
            self.instruction_cb, 
            10
        )

        self.cmd_angle_sub = self.create_subscription(
            Float32, 
            '/detection/tracking_angle', 
            self.tracking_angle_cb, 
            10
        )
        self.tof_sub = message_filters.Subscriber(
            self, 
            Range, 
            '/tof/distance', 
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info("Movement Node ready with PID control. Awaiting /cmd_movement...")

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]

    # --- FIXED: ADDED MISSING CALLBACK ---
    def instruction_cb(self, msg):
        """Processes the [distance, angle] command array."""
        if len(msg.data) < 2:
            self.get_logger().error("Instruction needs at least 2 values: [dist, angle]")
            return

        dist = msg.data[0]
        angle = msg.data[1]

        if dist != 0.0:
            self.get_logger().info(f"PID Linear Move: {dist}m")
            self.execute_linear_move(dist)
        elif angle != 0.0:
            self.get_logger().info(f"Encoder Rotation: {angle}deg")
            self.execute_rotation(angle)

    def execute_linear_move(self, distance):
        direction = 1 if distance < 0 else -1
        self.set_wheel_state(direction, direction)

        m1_speed, m2_speed = 0.3, 0.3
        e1_prev_err, e2_prev_err = 0, 0
        e1_sum_err, e2_sum_err = 0, 0
        
        total_dist = 0.0
        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        while rclpy.ok() and abs(total_dist) < abs(distance):
            t1_start = self.left_encoder._ticks
            t2_start = self.right_encoder._ticks

            time.sleep(self.SAMPLETIME)

            e1_val = abs(self.left_encoder._ticks - t1_start)
            e2_val = abs(self.right_encoder._ticks - t2_start)

            e1_err = self.TARGET - e1_val
            e2_err = self.TARGET - e2_val

            m1_speed += (e1_err * self.KP) + (e1_prev_err * self.KD) + (e1_sum_err * self.KI)
            m2_speed += (e2_err * self.KP) + (e2_prev_err * self.KD) + (e2_sum_err * self.KI)

            m1_speed = max(min(1.0, m1_speed), 0.0)
            m2_speed = max(min(1.0, m2_speed), 0.0)

            self.motor.set_wheels_speed(m1_speed * direction, m2_speed * direction)

            e1_prev_err, e2_prev_err = e1_err, e2_err
            e1_sum_err += e1_err
            e2_sum_err += e2_err

            avg_ticks = (abs(self.left_encoder._ticks) + abs(self.right_encoder._ticks)) / 2.0
            total_dist = (avg_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius

        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))

    def execute_rotation(self, degrees, speed=0.3):
        target_rad = abs(degrees) * (math.pi / 180.0)
        if degrees > 0: # Right
            self.set_wheel_state(1, -1)
            self.motor.set_wheels_speed(speed, -speed)
        else: # Left
            self.set_wheel_state(-1, 1)
            self.motor.set_wheels_speed(-speed, speed)

        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        while rclpy.ok():
            l_dist = (abs(self.left_encoder._ticks) / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            r_dist = (abs(self.right_encoder._ticks) / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            if abs((r_dist + l_dist) / self.axle_width) >= target_rad:
                break
            time.sleep(0.01)

    def tracking_angle_cb(self, msg):
        pass

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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()