#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from jetson_camera.imu_driver import Mpu6050
from jetson_camera.plot_route import yaw_to_quaternion


class ImuNode(Node):
    """Publish MPU6050 raw IMU data and integrated relative yaw with auto-recovery."""

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x68)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('calibration_samples', 400)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('yaw_topic', '/imu/yaw')
        self.declare_parameter('gyro_deadband_deg_s', 0.15)

        self.frame_id = self.get_parameter('frame_id').value
        self.gyro_deadband_deg_s = float(
            self.get_parameter('gyro_deadband_deg_s').value
        )
        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.device_address = int(self.get_parameter('device_address').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)

        # Failure tracking parameters
        self.consecutive_failures = 0
        self.max_allowed_failures = 8  # ~200ms of downtime at 40Hz before resetting
        self.calibrated = False
        self.gyro_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.sensor = None
        self.initialize_sensor()

        if self.sensor is not None:
            self.gyro_offset = self.calibrate_gyro(self.calibration_samples)
            self.calibrated = True

        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter('imu_topic').value,
            10,
        )
        self.yaw_pub = self.create_publisher(
            Float32,
            self.get_parameter('yaw_topic').value,
            10,
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_imu)

        self.get_logger().info(
            'IMU node ready. Keep the robot still during startup calibration.'
        )

    def initialize_sensor(self):
        """Attempts to open the bus and configure the MPU6050 hardware."""
        try:
            self.get_logger().info(f"Connecting to MPU6050 (I2C Bus: {self.i2c_bus}, Address: 0x{self.device_address:02x})...")
            self.sensor = Mpu6050(
                address=self.device_address,
                bus=self.i2c_bus,
            )
            self.sensor.set_accel_range(Mpu6050.ACCEL_RANGE_2G)
            self.sensor.set_gyro_range(Mpu6050.GYRO_RANGE_250DEG)
            self.sensor.set_filter_range(Mpu6050.FILTER_BW_188)
            self.get_logger().info("MPU6050 I2C connection initialized successfully.")
            return True
        except Exception as e:
            self.get_logger().error(f"MPU6050 initialization failed: {e}")
            self.sensor = None
            return False

    def calibrate_gyro(self, samples):
        self.get_logger().info('Calibrating gyro with %d samples...' % samples)
        sums = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        for _ in range(samples):
            try:
                gyro = self.sensor.get_gyro_data()
            except Exception:
                gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            sums['x'] += gyro['x']
            sums['y'] += gyro['y']
            sums['z'] += gyro['z']
            time.sleep(0.005)

        offset = {
            'x': sums['x'] / samples,
            'y': sums['y'] / samples,
            'z': sums['z'] / samples,
        }
        self.get_logger().info(
            'Gyro offset: x=%.3f y=%.3f z=%.3f deg/s'
            % (offset['x'], offset['y'], offset['z'])
        )
        return offset

    def recover_sensor(self):
        """Auto-recovery sequence to clear a locked I2C bus and reconnect the chip."""
        self.get_logger().error("Consecutive I2C failures exceeded threshold. Initiating auto-recovery sequence...")
        
        # 1. Gracefully close the stale file handle
        self.get_logger().info("Closing stale SMBus file descriptor...")
        try:
            if self.sensor is not None:
                self.sensor.close()
        except Exception as e:
            self.get_logger().warn(f"Failed to close SMBus cleanly: {e}")
        
        self.sensor = None
        
        # 2. Settle time to allow hardware lines to discharge/release
        time.sleep(0.15)
        
        # 3. Re-open connection
        self.get_logger().info("Attempting to reconnect and re-initialize MPU6050...")
        success = self.initialize_sensor()
        if success:
            self.get_logger().info("Re-initialization successful! Clearing error counters.")
            self.consecutive_failures = 0
            # If the sensor was never calibrated successfully at boot, run calibration now
            if not self.calibrated:
                self.gyro_offset = self.calibrate_gyro(self.calibration_samples)
                self.calibrated = True
            self.last_time = self.get_clock().now()  # Reset step timer
        else:
            self.get_logger().warn("Re-initialization failed. Will retry on next loop tick.")

    def publish_imu(self):
        # If the sensor is completely disconnected or initialization failed, attempt to connect
        if self.sensor is None:
            success = self.initialize_sensor()
            if success:
                if not self.calibrated:
                    self.gyro_offset = self.calibrate_gyro(self.calibration_samples)
                    self.calibrated = True
                self.last_time = self.get_clock().now()
            else:
                return  # Skip this tick until sensor is re-established

        try:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            accel = self.sensor.get_accel_data()
            gyro_deg_s = self.sensor.get_gyro_data()

            gyro_z_deg_s = gyro_deg_s['z'] - self.gyro_offset['z']
            if abs(gyro_z_deg_s) < self.gyro_deadband_deg_s:
                gyro_z_deg_s = 0.0

            gyro_rad_s = {
                'x': math.radians(gyro_deg_s['x'] - self.gyro_offset['x']),
                'y': math.radians(gyro_deg_s['y'] - self.gyro_offset['y']),
                'z': math.radians(gyro_z_deg_s),
            }

            self.yaw = self.normalize_angle(self.yaw + gyro_rad_s['z'] * dt)

            # Reset consecutive failures on any successful I2C read
            self.consecutive_failures = 0

            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.orientation = yaw_to_quaternion(self.yaw)
            imu_msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, 0.05,
            ]
            imu_msg.angular_velocity.x = gyro_rad_s['x']
            imu_msg.angular_velocity.y = gyro_rad_s['y']
            imu_msg.angular_velocity.z = gyro_rad_s['z']
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            imu_msg.linear_acceleration.x = accel['x']
            imu_msg.linear_acceleration.y = accel['y']
            imu_msg.linear_acceleration.z = accel['z']
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1,
            ]
            self.imu_pub.publish(imu_msg)

            yaw_msg = Float32()
            yaw_msg.data = float(self.yaw)
            self.yaw_pub.publish(yaw_msg)

        except Exception as exc:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f"IMU read failed (Failure count: {self.consecutive_failures}/{self.max_allowed_failures}): {exc}"
            )
            
            # If failures exceed limit, execute the reset sequence
            if self.consecutive_failures >= self.max_allowed_failures:
                self.recover_sensor()

    def shutdown(self):
        if self.sensor is not None:
            self.sensor.close()

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()