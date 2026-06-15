#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ImuYawAccuracyTest(Node):
    def __init__(self):
        super().__init__('imu_yaw_accuracy_test')

        self.declare_parameter('yaw_topic', '/imu/yaw')
        self.declare_parameter('calibration_samples', 50)

        self.calibration_samples = int(
            self.get_parameter('calibration_samples').value
        )
        self.yaw_samples = []
        self.zero_yaw = None
        self.sample_count = 0

        yaw_topic = self.get_parameter('yaw_topic').value
        self.create_subscription(Float32, yaw_topic, self.yaw_cb, 10)

        self.get_logger().info(
            'Keep the robot still. Calibrating zero yaw with %d samples...'
            % self.calibration_samples
        )

    def yaw_cb(self, msg):
        yaw = float(msg.data)

        if self.zero_yaw is None:
            self.yaw_samples.append(yaw)

            if len(self.yaw_samples) >= self.calibration_samples:
                self.zero_yaw = self.circular_mean(self.yaw_samples)
                self.get_logger().info(
                    'Calibration done. Zero yaw = %.4f rad / %.2f deg'
                    % (self.zero_yaw, math.degrees(self.zero_yaw))
                )
                print('reference_deg,measured_deg,error_deg')
            return

        relative_yaw = self.normalize_angle(yaw - self.zero_yaw)
        relative_deg = math.degrees(relative_yaw)
        self.sample_count += 1

        print(
            'sample=%d yaw_rad=%.4f yaw_deg=%.2f'
            % (self.sample_count, relative_yaw, relative_deg),
            flush=True,
        )

    @staticmethod
    def circular_mean(angles):
        sin_sum = sum(math.sin(angle) for angle in angles)
        cos_sum = sum(math.cos(angle) for angle in angles)
        return math.atan2(sin_sum, cos_sum)

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawAccuracyTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
