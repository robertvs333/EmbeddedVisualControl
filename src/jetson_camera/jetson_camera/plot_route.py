#!/usr/bin/env python3

import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Int64


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def direction_from_msg(msg):
    if msg.data > 0:
        return 1
    if msg.data < 0:
        return -1
    return 0


class RoutePlotter(Node):
    """Integrate wheel encoder ticks into a route and publish it live."""

    def __init__(self):
        super().__init__('route_plotter')

        self.declare_parameter('left_ticks_topic', '/encoders/left_ticks')
        self.declare_parameter('right_ticks_topic', '/encoders/right_ticks')
        self.declare_parameter('left_direction_topic', '/motors/left_direction')
        self.declare_parameter('right_direction_topic', '/motors/right_direction')
        self.declare_parameter('imu_yaw_topic', '/imu/yaw')
        self.declare_parameter('heading_source', 'encoder')
        self.declare_parameter('imu_heading_weight', 0.9)
        self.declare_parameter('path_topic', '/route/path')
        self.declare_parameter('odom_topic', '/route/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('ticks_per_rev', 140.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('ticks_are_signed', True)
        self.declare_parameter('save_plot_on_shutdown', True)
        self.declare_parameter('plot_output_path', 'route_plot.png')
        self.declare_parameter('save_route_log_on_shutdown', True)
        self.declare_parameter('route_log_output_path', 'route_log.csv')
        self.declare_parameter('show_live_plot', False)
        self.declare_parameter('update_rate_hz', 10.0)

        self.left_ticks_topic = self.get_parameter('left_ticks_topic').value
        self.right_ticks_topic = self.get_parameter('right_ticks_topic').value
        self.left_direction_topic = self.get_parameter('left_direction_topic').value
        self.right_direction_topic = self.get_parameter('right_direction_topic').value
        self.imu_yaw_topic = self.get_parameter('imu_yaw_topic').value
        self.heading_source = self.get_parameter('heading_source').value
        self.imu_heading_weight = float(self.get_parameter('imu_heading_weight').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.ticks_are_signed = bool(self.get_parameter('ticks_are_signed').value)
        self.save_plot_on_shutdown = bool(
            self.get_parameter('save_plot_on_shutdown').value
        )
        self.plot_output_path = self.get_parameter('plot_output_path').value
        self.save_route_log_on_shutdown = bool(
            self.get_parameter('save_route_log_on_shutdown').value
        )
        self.route_log_output_path = self.get_parameter('route_log_output_path').value
        self.show_live_plot = bool(self.get_parameter('show_live_plot').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.latest_left_ticks = None
        self.latest_right_ticks = None
        self.latest_imu_yaw = None
        self.initial_imu_yaw = None
        self.left_direction = 1
        self.right_direction = 1
        self.x_positions = [self.x]
        self.y_positions = [self.y]
        self.route_log = []

        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.frame_id

        self.path_pub = self.create_publisher(
            PathMsg,
            self.get_parameter('path_topic').value,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10,
        )

        self.create_subscription(
            Int64,
            self.left_ticks_topic,
            self.left_ticks_cb,
            10,
        )
        self.create_subscription(
            Int64,
            self.right_ticks_topic,
            self.right_ticks_cb,
            10,
        )
        self.create_subscription(
            Int8,
            self.left_direction_topic,
            self.left_direction_cb,
            10,
        )
        self.create_subscription(
            Int8,
            self.right_direction_topic,
            self.right_direction_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.imu_yaw_topic,
            self.imu_yaw_cb,
            10,
        )
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.update_timer = self.create_timer(1.0 / update_rate_hz, self.update_route)

        self.figure = None
        self.axis = None
        self.line = None
        if self.show_live_plot:
            self._init_live_plot()

        self.get_logger().info(
            'Route plotter ready. Heading source: %s. Subscribing to %s and %s.'
            % (self.heading_source, self.left_ticks_topic, self.right_ticks_topic)
        )

    def left_ticks_cb(self, msg):
        self.latest_left_ticks = int(msg.data)

    def right_ticks_cb(self, msg):
        self.latest_right_ticks = int(msg.data)

    def left_direction_cb(self, msg):
        self.left_direction = direction_from_msg(msg)

    def right_direction_cb(self, msg):
        self.right_direction = direction_from_msg(msg)

    def imu_yaw_cb(self, msg):
        self.latest_imu_yaw = float(msg.data)
        if self.initial_imu_yaw is None:
            self.initial_imu_yaw = self.latest_imu_yaw

    def update_route(self):
        if self.latest_left_ticks is None or self.latest_right_ticks is None:
            return

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.latest_left_ticks
            self.prev_right_ticks = self.latest_right_ticks
            self.append_route_log(0, 0, 0.0, 0.0, 0.0)
            self.publish_pose()
            return

        delta_left_ticks = self.latest_left_ticks - self.prev_left_ticks
        delta_right_ticks = self.latest_right_ticks - self.prev_right_ticks
        self.prev_left_ticks = self.latest_left_ticks
        self.prev_right_ticks = self.latest_right_ticks

        if not self.ticks_are_signed:
            delta_left_ticks = abs(delta_left_ticks) * self.left_direction
            delta_right_ticks = abs(delta_right_ticks) * self.right_direction

        if delta_left_ticks == 0 and delta_right_ticks == 0:
            return

        delta_left = self.ticks_to_distance(delta_left_ticks)
        delta_right = self.ticks_to_distance(delta_right_ticks)
        delta_center = (delta_right + delta_left) * 0.5
        delta_theta = (delta_right - delta_left) / self.wheel_base

        previous_theta = self.theta
        encoder_theta = self.normalize_angle(self.theta + delta_theta)
        new_theta = self.select_heading(encoder_theta)
        heading_midpoint = self.normalize_angle(
            previous_theta + self.shortest_angle(previous_theta, new_theta) * 0.5
        )

        self.x += delta_center * math.cos(heading_midpoint)
        self.y += delta_center * math.sin(heading_midpoint)
        self.theta = new_theta

        self.x_positions.append(self.x)
        self.y_positions.append(self.y)
        self.append_route_log(
            delta_left_ticks,
            delta_right_ticks,
            delta_left,
            delta_right,
            delta_theta,
            self.get_imu_theta(),
        )
        self.publish_pose()
        self.update_live_plot()

    def ticks_to_distance(self, ticks):
        return (ticks / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius

    def select_heading(self, encoder_theta):
        imu_theta = self.get_imu_theta()

        if self.heading_source == 'imu':
            if imu_theta is not None:
                return imu_theta
            return encoder_theta

        if self.heading_source == 'fused':
            if imu_theta is not None:
                weight = min(max(self.imu_heading_weight, 0.0), 1.0)
                return self.blend_angles(encoder_theta, imu_theta, weight)
            return encoder_theta

        return encoder_theta

    def get_imu_theta(self):
        if self.latest_imu_yaw is None or self.initial_imu_yaw is None:
            return None

        relative_imu_yaw = self.shortest_angle(
            self.initial_imu_yaw,
            self.latest_imu_yaw,
        )
        return self.normalize_angle(
            float(self.get_parameter('initial_theta').value) + relative_imu_yaw
        )

    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        orientation = yaw_to_quaternion(self.theta)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

    def append_route_log(
        self,
        delta_left_ticks,
        delta_right_ticks,
        delta_left,
        delta_right,
        delta_theta,
        imu_theta=None,
    ):
        self.route_log.append({
            'time_sec': self.get_clock().now().nanoseconds * 1e-9,
            'left_ticks': self.latest_left_ticks,
            'right_ticks': self.latest_right_ticks,
            'delta_left_ticks': delta_left_ticks,
            'delta_right_ticks': delta_right_ticks,
            'delta_left_m': delta_left,
            'delta_right_m': delta_right,
            'delta_theta_rad': delta_theta,
            'imu_theta_rad': imu_theta,
            'heading_source': self.heading_source,
            'x_m': self.x,
            'y_m': self.y,
            'theta_rad': self.theta,
        })

    def _init_live_plot(self):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning(
                'matplotlib is not installed; live plotting disabled.'
            )
            self.show_live_plot = False
            return

        plt.ion()
        self.figure, self.axis = plt.subplots()
        (self.line,) = self.axis.plot([], [], marker='o')
        self.axis.set_aspect('equal', adjustable='datalim')
        self.axis.set_xlabel('x position [m]')
        self.axis.set_ylabel('y position [m]')
        self.axis.set_title('Robot route from wheel encoders')
        self.axis.grid(True)

    def update_live_plot(self):
        if not self.show_live_plot or self.figure is None:
            return

        self.line.set_data(self.x_positions, self.y_positions)
        self.axis.relim()
        self.axis.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def save_plot(self):
        if len(self.x_positions) < 2:
            self.get_logger().info('Route plot not saved because no movement was recorded.')
            return

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning(
                'matplotlib is not installed; route image was not saved.'
            )
            return

        output_path = Path(os.path.expanduser(self.plot_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        figure, axis = plt.subplots()
        axis.plot(self.x_positions, self.y_positions, marker='o')
        axis.set_aspect('equal', adjustable='datalim')
        axis.set_xlabel('x position [m]')
        axis.set_ylabel('y position [m]')
        axis.set_title('Robot route from wheel encoders')
        axis.grid(True)
        figure.savefig(str(output_path), bbox_inches='tight')
        plt.close(figure)
        self.get_logger().info('Saved route plot to %s' % output_path)

    def save_route_log(self):
        if not self.route_log:
            self.get_logger().info('Route log not saved because no samples were recorded.')
            return

        import csv

        output_path = Path(os.path.expanduser(self.route_log_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        fieldnames = [
            'time_sec',
            'left_ticks',
            'right_ticks',
            'delta_left_ticks',
            'delta_right_ticks',
            'delta_left_m',
            'delta_right_m',
            'delta_theta_rad',
            'imu_theta_rad',
            'heading_source',
            'x_m',
            'y_m',
            'theta_rad',
        ]
        with output_path.open('w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.route_log)

        self.get_logger().info('Saved route log to %s' % output_path)

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def shortest_angle(cls, from_angle, to_angle):
        return cls.normalize_angle(to_angle - from_angle)

    @classmethod
    def blend_angles(cls, encoder_theta, imu_theta, imu_weight):
        return cls.normalize_angle(
            encoder_theta + cls.shortest_angle(encoder_theta, imu_theta) * imu_weight
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.save_route_log_on_shutdown:
            node.save_route_log()
        if node.save_plot_on_shutdown:
            node.save_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
