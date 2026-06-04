#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray
import math
import time

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        
        # Hardware Driver Initialization
        self.motor = DaguWheelsDriver()
        self.left_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_1)
        self.right_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_2)

        # Robot Physical Constants
        self.wheel_radius = 0.033   # Meters
        self.axle_width = 0.19      # Meters (Distance between wheels)
        self.encoder_resolution = 140.0
        
        # State tracking for "Publish on Change" logic
        self.last_wheel_state = [0, 0]

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.wheel_pub = self.create_publisher(Int32MultiArray, '/movement/wheel_status', 10)
        
        # Input: [distance_m, degrees] (Right is positive, Left is negative)
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 
            '/cmd_movement', 
            self.instruction_cb, 
            10
        )

        self.get_logger().info("Movement Node initialized and awaiting instructions.")

    def set_wheel_state(self, left, right):
        """Publishes 1 (Fwd), -1 (Bwd), or 0 (Stop) to Mapping node only if state changes."""
        new_state = [int(left), int(right)]
        if new_state != self.last_wheel_state:
            msg = Int32MultiArray()
            msg.data = new_state
            self.wheel_pub.publish(msg)
            self.last_wheel_state = new_state

    def instruction_cb(self, msg):
        """Main callback to process movement sequences from the Algorithm node."""
        if len(msg.data) < 2:
            self.get_logger().error("Invalid command array received. Expected [distance, angle].")
            return

        target_dist = msg.data[0]
        target_deg = msg.data[1]
        
        # Clear encoders for the next precise operation
        self.left_encoder._ticks = 0
        self.right_encoder._ticks = 0

        if target_dist != 0.0:
            self.get_logger().info(f"Moving {target_dist}m...")
            self.execute_linear_move(target_dist)
        elif target_deg != 0.0:
            self.get_logger().info(f"Rotating {target_deg} degrees...")
            self.execute_rotation(target_deg)

        # Ensure motors stop and notify Algorithm the task is complete
        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))

    def execute_linear_move(self, distance, speed=0.3):
        """Drives in a straight line until encoders match target distance."""
        direction = 1 if distance > 0 else -1
        self.set_wheel_state(direction, direction)
        self.motor.set_wheels_speed(speed * direction, speed * direction)
        
        while rclpy.ok():
            # Calculate distance traveled per wheel
            l_m = abs((self.left_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius)
            r_m = abs((self.right_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius)
            
            # Use average distance for linear precision
            if (l_m + r_m) / 2.0 >= abs(distance):
                break
            time.sleep(0.01)

    def execute_rotation(self, degrees, speed=0.3):
        """Rotates based on differential wheel distance (Arc Length)."""
        target_rad = abs(degrees) * (math.pi / 180.0)
        
        # Right turns are positive, Left turns are negative
        if degrees > 0: # Turn Right
            self.set_wheel_state(1, -1) # Left Forward, Right Backward
            self.motor.set_wheels_speed(speed, -speed)
        else: # Turn Left
            self.set_wheel_state(-1, 1) # Left Backward, Right Forward
            self.motor.set_wheels_speed(-speed, speed)

        while rclpy.ok():
            # Calculate actual rotation angle (rad) = (RightDist - LeftDist) / AxleWidth
            l_dist = (self.left_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            r_dist = (self.right_encoder._ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            
            current_rot_rad = abs((r_dist - l_dist) / self.axle_width)
            
            if current_rot_rad >= target_rad:
                break
            time.sleep(0.01)

    def destroy_node(self):
        """Safe shutdown of hardware resources."""
        self.motor.set_wheels_speed(0.0, 0.0)
        self.motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()