#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

class SquareDriverNode(Node):
    def __init__(self):
        super().__init__('square_driver_node')
        
        # Define the exact 50cm x 50cm square pattern path.
        # Alternates strictly: [Distance (meters), Angle (degrees)]
        self.sequence = [
            [0.5, 0.0],   # Move Forward 50cm (Side 1)
            [0.0, 90.0],  # Turn Left 90 degrees
            [0.5, 0.0],   # Move Forward 50cm (Side 2)
            [0.0, 90.0],  # Turn Left 90 degrees
            [0.5, 0.0],   # Move Forward 50cm (Side 3)
            [0.0, 90.0],  # Turn Left 90 degrees
            [0.5, 0.0],   # Move Forward 50cm (Side 4)
            [0.0, 90.0]   # Turn Left 90 degrees (Restores original orientation)
        ]
        
        self.current_step = 0
        self.sleep_delay = 1.0  # Configurable delay (seconds) between receiving 'ready' and sending new data
        
        # Communication Infrastructure
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_movement', 10)
        self.finished_sub = self.create_subscription(Bool, '/movement_finished', self.movement_finished_cb, 10)
        
        # Variable to hold our dynamic delay timers
        self.delay_timer = None
        
        # Give the system 2 seconds to initialize properly, then begin the sequence
        self.get_logger().info("Square Driver initialized. Starting sequence in 2 seconds...")
        self.start_timer = self.create_timer(2.0, self.start_sequence)

    def start_sequence(self):
        # Destroy the initialization timer so it only runs once
        self.destroy_timer(self.start_timer)
        self.get_logger().info("Executing square sequence pattern!")
        self.send_current_command()

    def movement_finished_cb(self, msg):
        # Only act if the incoming execution flag indicates a complete execution state (True)
        if msg.data:
            self.get_logger().info(f"Step {self.current_step + 1}/{len(self.sequence)} completed successfully.")
            self.current_step += 1
            
            # Check if there are remaining instructions in the array
            if self.current_step < len(self.sequence):
                self.get_logger().info(f"Settle buffer: Sleeping for {self.sleep_delay} seconds...")
                
                # Create a non-blocking one-shot timer to handle the sleep interval
                self.delay_timer = self.create_timer(self.sleep_delay, self.delay_expiry_cb)
            else:
                self.get_logger().info("Success! The robot completed the full 50cm x 50cm square. Automatically stopping node...")
                
                # Initiate a clean ROS2 system shutdown to stop rclpy.spin() and exit the terminal process
                rclpy.shutdown()

    def delay_expiry_cb(self):
        # Tear down the one-shot timer so it doesn't fire repeatedly
        if self.delay_timer is not None:
            self.destroy_timer(self.delay_timer)
            self.delay_timer = None
            
        # Send the next queued instruction profile
        self.send_current_command()

    def send_current_command(self):
        # Safety check to ensure we don't index out of bounds if a message arrives during shutdown
        if self.current_step >= len(self.sequence):
            return

        # Extract the parameters for the current step index
        step = self.sequence[self.current_step]
        distance = float(step[0])
        angle = float(step[1])
        
        # Construct the payload message matching your movement node's interface
        msg = Float32MultiArray()
        msg.data = [distance, angle]
        
        # Diagnostic logging
        if distance != 0.0:
            self.get_logger().info(f"Publishing Linear Command: Drive {distance} meters (Turn parameter locked at 0.0)")
        else:
            self.get_logger().info(f"Publishing Rotational Command: Turn {angle} degrees (Distance parameter locked at 0.0)")
            
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Check if the node hasn't already been shut down by the internal success routine
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
