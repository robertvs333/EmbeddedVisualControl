#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int64

from jetson_camera.motorDrivers.encoderDriver import (
    WheelDirection,
    WheelEncoderDriver,
)


def wheel_direction_from_msg(msg, previous_direction):
    if msg.data > 0:
        return WheelDirection.FORWARD
    if msg.data < 0:
        return WheelDirection.REVERSE
    return previous_direction


class EncoderNode(Node):
    """Read both wheel encoders and publish cumulative signed ticks."""

    def __init__(self):
        super().__init__('encoder_node')

        self.declare_parameter('left_encoder_pin', 35)
        self.declare_parameter('right_encoder_pin', 12)
        self.declare_parameter('left_ticks_topic', '/encoders/left_ticks')
        self.declare_parameter('right_ticks_topic', '/encoders/right_ticks')
        self.declare_parameter('left_direction_topic', '/motors/left_direction')
        self.declare_parameter('right_direction_topic', '/motors/right_direction')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.left_encoder = WheelEncoderDriver(
            int(self.get_parameter('left_encoder_pin').value)
        )
        self.right_encoder = WheelEncoderDriver(
            int(self.get_parameter('right_encoder_pin').value)
        )

        self.left_pub = self.create_publisher(
            Int64,
            self.get_parameter('left_ticks_topic').value,
            10,
        )
        self.right_pub = self.create_publisher(
            Int64,
            self.get_parameter('right_ticks_topic').value,
            10,
        )

        self.create_subscription(
            Int8,
            self.get_parameter('left_direction_topic').value,
            self.left_direction_cb,
            10,
        )
        self.create_subscription(
            Int8,
            self.get_parameter('right_direction_topic').value,
            self.right_direction_cb,
            10,
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_ticks)

        self.get_logger().info('Encoder node ready. Publishing cumulative signed ticks.')

    def left_direction_cb(self, msg):
        direction = wheel_direction_from_msg(msg, self.left_encoder.get_direction())
        self.left_encoder.set_direction(direction)

    def right_direction_cb(self, msg):
        direction = wheel_direction_from_msg(msg, self.right_encoder.get_direction())
        self.right_encoder.set_direction(direction)

    def publish_ticks(self):
        left_msg = Int64()
        right_msg = Int64()
        left_msg.data = int(self.left_encoder._ticks)
        right_msg.data = int(self.right_encoder._ticks)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def shutdown(self):
        self.left_encoder.shutdown()
        self.right_encoder.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

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
