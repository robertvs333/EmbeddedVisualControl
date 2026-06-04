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
        
        # PID Constants 
        # Reduced KP and KI to prevent rapid speed runaway
        self.SAMPLETIME = 0.1 
        self.TARGET = 20       # Target ticks per 0.1s (Adjust this for cruise speed)
        self.KP = 0.004        
        self.KD = 0.0025       
        self.KI = 0.0005       

        self.last_wheel_state = [0, 0]

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 
            '/cmd_movement', 
            self.instruction_cb, 
            10
        )

        self.get_logger().info("Movement Node ready (Reversed Orientation + PID Tuned).")

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]

    def instruction_cb(self, msg):
        if len(msg.data) < 2:
            return
        dist, angle = msg.data[0], msg.data[1]

        if dist != 0.0:
            self.execute_linear_move(dist)
        elif angle != 0.0:
            self.execute_rotation(angle)

    def execute_linear_move(self, distance):
        # SWAP ORIENTATION: distance > 0 now sends direction -1 (Backwards)
        direction = -1 if distance > 0 else 1
        self.set_wheel_state(direction, direction)

        # Start at a lower base speed
        m1_speed, m2_speed = 0.2, 0.2
        e1_prev_err, e2_prev_err = 0, 0
        e1_sum_err, e2_sum_err = 0, 0
        
        total_dist = 0.0
        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        while rclpy.ok() and abs(total_dist) < abs(distance):
            t1_start = self.left_encoder._ticks
            t2_start = self.right_encoder._ticks

            time.sleep(self.SAMPLETIME)

            # Get absolute ticks moved during the sample
            e1_val = abs(self.left_encoder._ticks - t1_start)
            e2_val = abs(self.right_encoder._ticks - t2_start)

            # Error calculation
            e1_err = self.TARGET - e1_val
            e2_err = self.TARGET - e2_val

            # PID Velocity control: adds a small adjustment to current speed
            m1_adj = (e1_err * self.KP) + ((e1_err - e1_prev_err) * self.KD) + (e1_sum_err * self.KI)
            m2_adj = (e2_err * self.KP) + ((e2_err - e2_prev_err) * self.KD) + (e2_sum_err * self.KI)

            m1_speed += m1_adj
            m2_speed += m2_adj

            # Clamp between 0.1 (min to keep moving) and 0.8 (max for stability)
            m1_speed = max(min(0.8, m1_speed), 0.1)
            m2_speed = max(min(0.8, m2_speed), 0.1)

            self.motor.set_wheels_speed(m1_speed * direction, m2_speed * direction)
            
            # Updates for next iteration
            e1_prev_err = e1_err
            e2_prev_err = e2_err
            e1_sum_err += e1_err
            
            # Anti-windup: limit the sum error to stop runaway
            e1_sum_err = max(min(e1_sum_err, 50), -50)
            e2_sum_err += e2_err
            e2_sum_err = max(min(e2_sum_err, 50), -50)

            avg_ticks = (abs(self.left_encoder._ticks) + abs(self.right_encoder._ticks)) / 2.0
            total_dist = (avg_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius

        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))

    def execute_rotation(self, degrees, speed=0.3):
        target_rad = abs(degrees) * (math.pi / 180.0)
        
        # When swapping front/back, you may need to swap Left/Right logic here too
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