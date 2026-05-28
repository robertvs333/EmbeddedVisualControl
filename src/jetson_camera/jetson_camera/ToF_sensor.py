#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

# Try to import your custom driver file from the same directory
try:
    from jetson_camera.Driver.ToFDriver import VL53L0X
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

class TofPublisherNode(Node):
    def __init__(self):
        super().__init__('tof_publisher_node')
        self.get_logger().info("Initializing Clean ToF Sensor Node...")

        # 1. Create Publisher matching the Sensor Data Profile
        self.tof_pub = self.create_publisher(Range, '/tof/distance', qos_profile=qos_profile_sensor_data)

        self.hardware_available = HARDWARE_AVAILABLE
        self.sensor = None

        if self.hardware_available:
            try:
                # Access the driver class directly from your ToFDriver file
                self.sensor = VL53L0X(bus=1, address=0x29)
                self.get_logger().info("VL53L0X hardware linked successfully from ToFDriver!")
            except Exception as e:
                self.get_logger().error(f"Failed to talk to sensor via ToFDriver: {e}")
                self.hardware_available = False
        
        if not self.hardware_available:
            self.get_logger().warn("Running in SIMULATED MOCK MODE (ToFDriver.py missing or disconnected).")

        # 2. High-Frequency Timer Loop (30Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        distance_meters = 0.0

        if self.hardware_available and self.sensor:
            # Call the distance read method directly from your driver file
            distance_mm = self.sensor.read_distance()
            
            if distance_mm is not None and distance_mm > 0:
                distance_meters = float(distance_mm) / 1000.0
            else:
                return  # Skip temporary dropped readings cleanly
        else:
            distance_meters = 1.25  # Simulated baseline data for bench testing

        # 3. Construct and Publish standard ROS 2 Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'tof_sensor_link'
        
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.44  # ~25 degrees FoV for VL53L0X
        range_msg.min_range = 0.03      # 3 cm min
        range_msg.max_range = 2.0       # 2.0 meters max
        range_msg.range = distance_meters

        self.tof_pub.publish(range_msg)

    def shutdown_node(self):
        if self.sensor:
            try:
                self.sensor.close()
                self.get_logger().info("I2C Bus closed cleanly.")
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TofPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ToF node...")
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()