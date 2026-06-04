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
        
        # Keep track of state to only publish when it changes
        self.last_wheel_state = [0, 0]

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        # Subscriber for instructions: [distance, degrees]
        # self.cmd_sub = self.create_subscription(
        #     Float32MultiArray, 
        #     '/cmd_movement', 
        #     self.instruction_cb, 
        #     10
        # )
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

        self.get_logger().info("Movement Node ready for split direction publishing.")

    def set_wheel_state(self, left, right):
        """Publishes 1, -1, or 0 to separate topics only if the direction changes."""
        if [left, right] != self.last_wheel_state:
            # Create Int8 messages
            l_msg = Int8()
            l_msg.data = int(left)
            r_msg = Int8()
            r_msg.data = int(right)
            
            # Publish to separate topics
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            
            self.last_wheel_state = [left, right]

    def instruction(self, msg):
        if len(msg.data) < 2:
            return

        target_dist = self.tof_sub
        target_deg =  (180*self.cmd_angle_sub)/math.pi
        
        # Reset ticks for fresh measurement
        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        if target_dist != 0.0:
            self.execute_linear_move(target_dist)
        elif target_deg != 0.0:
            self.execute_rotation(target_deg)

        # Stop hardware and notify system
        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))

    def execute_linear_move(self, distance, speed=0.3):
        direction = 1 if distance > 0 else -1
        self.set_wheel_state(direction, direction)
        self.motor.set_wheels_speed(speed * direction, speed * direction)
        
        while rclpy.ok():
            # Encoder distance math
            l_m = abs((self.left_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius)
            r_m = abs((self.right_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius)
            
            if (l_m + r_m) / 2.0 >= abs(distance):
                break
            time.sleep(0.01)

    def execute_rotation(self, degrees, speed=0.3):
        target_rad = abs(degrees) * (math.pi / 180.0)
        
        # Positive = Right, Negative = Left
        if degrees > 0: # Turn Right
            self.set_wheel_state(1, -1) # Left Fwd, Right Bwd
            self.motor.set_wheels_speed(speed, -speed)
        else: # Turn Left
            self.set_wheel_state(-1, 1) # Left Bwd, Right Fwd
            self.motor.set_wheels_speed(-speed, speed)

        while rclpy.ok():
            # (RightDist - LeftDist) / AxleWidth = Angle in Radians
            l_dist = (self.left_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            r_dist = (self.right_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            
            if abs((r_dist - l_dist) / self.axle_width) >= target_rad:
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