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
    """Publish MPU6050 raw IMU data and integrated relative yaw."""

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x68)
        self.declare_parameter('publish_rate_hz', 40.0)
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('yaw_topic', '/imu/yaw')
        self.declare_parameter('gyro_deadband_deg_s', 0.15)

        self.frame_id = self.get_parameter('frame_id').value
        self.gyro_deadband_deg_s = float(
            self.get_parameter('gyro_deadband_deg_s').value
        )

        self.sensor = Mpu6050(
            address=int(self.get_parameter('device_address').value),
            bus=int(self.get_parameter('i2c_bus').value),
        )
        self.sensor.set_accel_range(Mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(Mpu6050.GYRO_RANGE_250DEG)
        self.sensor.set_filter_range(Mpu6050.FILTER_BW_42)

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

        self.gyro_offset = self.calibrate_gyro(
            int(self.get_parameter('calibration_samples').value)
        )
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_imu)

        self.get_logger().info(
            'IMU node ready. Keep the robot still during startup calibration.'
        )

    def calibrate_gyro(self, samples):
        self.get_logger().info('Calibrating gyro with %d samples...' % samples)
        sums = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        for _ in range(samples):
            gyro = self.sensor.get_gyro_data()
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

    def publish_imu(self):
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
            self.get_logger().warn('IMU read failed: %s' % exc)

    def shutdown(self):
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
