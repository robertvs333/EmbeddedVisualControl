#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

# --- HARDWARE IMPORT PLACEHOLDER ---
# Replace this with your specific sensor library (e.g., import qwiic_vl53l1x or import VL53L1X)
# For this example, we will assume a standard I2C ToF driver structure.
try:
    import VL53L1X
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
# -----------------------------------

class TofPublisherNode(Node):
    def __init__(self):
        super().__init__('tof_publisher_node')
        self.get_logger().info("Initializing Time-of-Flight Sensor Node...")

        # 1. Create Publisher matching the Sensor Data Profile for performance
        self.tof_pub = self.create_publisher(Range, '/tof/distance', qos_profile=qos_profile_sensor_data)

        # 2. Hardware Initialization Guard
        self.hardware_available = HARDWARE_AVAILABLE
        if self.hardware_available:
            try:
                # Initialize I2C bus and sensor
                self.sensor = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
                self.sensor.open()
                self.sensor.start_ranging(3) # Short/Medium/Long range mode selection
                self.get_logger().info("ToF hardware initialized on I2C Bus 1.")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to ToF Hardware via I2C: {e}")
                self.hardware_available = False
        else:
            self.get_logger().info("VL53L1X library not available; using simulated mock mode.")

        if not self.hardware_available:
            self.get_logger().warn("Hardware driver missing or disconnected. Running in SIMULATED MOCK MODE.")

        # 3. Create High-Frequency Timer Loop (30Hz)
        # 30Hz matches your camera frame rate, optimizing synchronizer time-matching on the laptop
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        """High-speed polling loop to read the physical distance and publish standard ROS messages."""
        distance_meters = 0.0

        if self.hardware_available:
            try:
                # Read hardware value (typically returned in millimeters)
                distance_mm = self.sensor.get_distance() 
                
                # Convert millimeters to meters for standard ROS spatial compliance
                distance_meters = float(distance_mm) / 1000.0
            except Exception as e:
                self.get_logger().error(f"Error reading I2C stream from ToF sensor: {e}")
                return
        else:
            # Simulated Data fallback for bench testing without the physical sensor wired up
            distance_meters = 1.25 

        # 4. Construct the standard ROS 2 Range Message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'tof_sensor_link'
        
        # Populate sensor specifications
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.47  # ~27 degrees Field of View (typical for VL53L1X)
        range_msg.min_range = 0.04      # 4 cm minimum tracking range
        range_msg.max_range = 4.0       # 4.0 meters maximum tracking range
        
        # Assign the actual reading
        range_msg.range = distance_meters

        # Broadcast payload
        self.tof_pub.publish(range_msg)

    def shutdown_node(self):
        """Safely shuts down the I2C registers to prevent bus lockups."""
        if self.hardware_available:
            try:
                self.sensor.stop_ranging()
                self.get_logger().info("ToF laser ranging halted cleanly.")
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TofPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ToF publisher...")
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()