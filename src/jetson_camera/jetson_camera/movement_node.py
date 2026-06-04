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
        
        # PID Constants (Based on PDF suggestions)
        self.SAMPLETIME = 0.1  # 10Hz updates for better responsiveness
        self.TARGET = 25       # Target ticks per sample interval
        self.KP = 0.02
        self.KD = 0.01
        self.KI = 0.005

        self.last_wheel_state = [0, 0]

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
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

        self.get_logger().info("Movement Node ready with PID straight-line control.")

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]

    def execute_linear_move(self, distance):
        """Drives straight using PID control as described in the tutorial."""
        direction = 1 if distance > 0 else -1
        self.set_wheel_state(direction, direction)

        # PID Variables
        m1_speed = 0.3  # Starting speed (Left)
        m2_speed = 0.3  # Starting speed (Right)
        
        e1_prev_error = 0
        e2_prev_error = 0
        e1_sum_error = 0
        e2_sum_error = 0

        total_dist_traveled = 0.0
        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        while rclpy.ok() and abs(total_dist_traveled) < abs(distance):
            # 1. Capture current ticks for this sample
            e1_ticks_start = self.left_encoder._ticks
            e2_ticks_start = self.right_encoder._ticks

            time.sleep(self.SAMPLETIME)

            # 2. Calculate how many ticks occurred during the sample (Actual Speed)
            e1_value = abs(self.left_encoder._ticks - e1_ticks_start)
            e2_value = abs(self.right_encoder._ticks - e2_ticks_start)

            # 3. Calculate Error (Target - Actual)
            e1_error = self.TARGET - e1_value
            e2_error = self.TARGET - e2_value

            # 4. PID Calculation: adjustment = (P) + (D) + (I)
            m1_adj = (e1_error * self.KP) + (e1_prev_error * self.KD) + (e1_sum_error * self.KI)
            m2_adj = (e2_error * self.KP) + (e2_prev_error * self.KD) + (e2_sum_error * self.KI)

            # 5. Apply adjustment to current speeds
            m1_speed += m1_adj
            m2_speed += m2_adj

            # 6. Clamp speeds between 0 and 1
            m1_speed = max(min(1.0, m1_speed), 0.0)
            m2_speed = max(min(1.0, m2_speed), 0.0)

            # 7. Update motors
            self.motor.set_wheels_speed(m1_speed * direction, m2_speed * direction)

            # 8. Update history for next loop
            e1_prev_error = e1_error
            e2_prev_error = e2_error
            e1_sum_error += e1_error
            e2_sum_error += e2_error

            # 9. Track total distance for exit condition
            avg_ticks = (abs(self.left_encoder._ticks) + abs(self.right_encoder._ticks)) / 2.0
            total_dist_traveled = (avg_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius

        # Stop and cleanup
        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))

    def execute_rotation(self, degrees, speed=0.3):
        # (Rotation logic remains standard as PID is primarily for straight lines)
        target_rad = abs(degrees) * (math.pi / 180.0)
        if degrees > 0:
            self.set_wheel_state(1, -1)
            self.motor.set_wheels_speed(speed, -speed)
        else:
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
        # Placeholder logic for tracking angle if needed
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