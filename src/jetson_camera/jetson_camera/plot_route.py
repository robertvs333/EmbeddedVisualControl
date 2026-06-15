#!/usr/bin/env python3

import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import Range 
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Int64

from datetime import datetime

# --- ADDED: Import QoS tools to fix connection profile compatibility ---
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Global placeholder for matplotlib functions
plt = None

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


def point_distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def segment_intersection(a, b, c, d):
    ab_x = b[0] - a[0]
    ab_y = b[1] - a[1]
    cd_x = d[0] - c[0]
    cd_y = d[1] - c[1]
    denominator = ab_x * cd_y - ab_y * cd_x

    if abs(denominator) < 1e-12:
        return None

    ca_x = c[0] - a[0]
    ca_y = c[1] - a[1]
    t = (ca_x * cd_y - ca_y * cd_x) / denominator
    u = (ca_x * ab_y - ca_y * ab_x) / denominator

    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (a[0] + t * ab_x, a[1] + t * ab_y)
    return None


def loop_erase_route(points, loop_closure_distance):
    if len(points) < 3:
        return points

    simplified = [points[0]]
    for point in points[1:]:
        if loop_closure_distance > 0.0:
            for index, old_point in enumerate(simplified[:-1]):
                if point_distance(point, old_point) <= loop_closure_distance:
                    simplified = simplified[:index + 1]
                    break

        if len(simplified) >= 2:
            segment_start = simplified[-1]
            segment_end = point
            for index in range(len(simplified) - 2):
                intersection = segment_intersection(
                    simplified[index],
                    simplified[index + 1],
                    segment_start,
                    segment_end,
                )
                if intersection is not None:
                    simplified = simplified[:index + 1]
                    simplified.append(intersection)
                    break

        if point_distance(simplified[-1], point) > 1e-9:
            simplified.append(point)

    return simplified


def perpendicular_distance(point, line_start, line_end):
    line_length = point_distance(line_start, line_end)
    if line_length < 1e-12:
        return point_distance(point, line_start)

    numerator = abs(
        (line_end[1] - line_start[1]) * point[0]
        - (line_end[0] - line_start[0]) * point[1]
        + line_end[0] * line_start[1]
        - line_end[1] * line_start[0]
    )
    return numerator / line_length


def ramer_douglas_peucker(points, tolerance):
    if len(points) < 3 or tolerance <= 0.0:
        return points

    max_distance = 0.0
    max_index = 0
    for index in range(1, len(points) - 1):
        distance = perpendicular_distance(points[index], points[0], points[-1])
        if distance > max_distance:
            max_distance = distance
            max_index = index

    if max_distance > tolerance:
        left = ramer_douglas_peucker(points[:max_index + 1], tolerance)
        right = ramer_douglas_peucker(points[max_index:], tolerance)
        return left[:-1] + right

    return [points[0], points[-1]]

class RoutePlotter(Node):
    """Integrate wheel encoder ticks into a route and publish it live."""

    def __init__(self):
        super().__init__('route_plotter')

        # --- ENVIRONMENT TOGGLE PARAMETER ---
        self.declare_parameter('run_on_jetson', True)

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
        # self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.185)
        self.declare_parameter('ticks_per_rev', 140.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('tof_topic', '/tof/distance')
        self.declare_parameter('ticks_are_signed', True)
        self.declare_parameter('save_plot_on_shutdown', True)
        self.declare_parameter('plot_output_path', 'route_plot.png')
        self.declare_parameter('save_route_log_on_shutdown', True)
        self.declare_parameter('route_log_output_path', 'route_log.csv')
        self.declare_parameter('show_live_plot', True)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('undefined_object_margin', 0.05)
        self.declare_parameter('save_simplified_route_on_shutdown', True)
        self.declare_parameter('simplified_plot_output_path', 'route_simplified.png')
        self.declare_parameter('simplified_route_log_output_path', 'route_simplified.csv')
        self.declare_parameter('loop_closure_distance', 0.05)
        self.declare_parameter('simplification_tolerance', 0.01)
        self.declare_parameter('route_overlay_plot_output_path', 'route_overlay.png')

        # Extract environment target
        self.run_on_jetson = bool(self.get_parameter('run_on_jetson').value)

        self.left_ticks_topic = self.get_parameter('left_ticks_topic').value
        self.right_ticks_topic = self.get_parameter('right_ticks_topic').value
        self.left_direction_topic = self.get_parameter('left_direction_topic').value
        self.right_direction_topic = self.get_parameter('right_direction_topic').value
        self.imu_yaw_topic = self.get_parameter('imu_yaw_topic').value
        self.heading_source = self.get_parameter('heading_source').value
        self.tof_topic = self.get_parameter('tof_topic').value
        self.imu_heading_weight = float(self.get_parameter('imu_heading_weight').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.ticks_are_signed = bool(self.get_parameter('ticks_are_signed').value)
        self.save_plot_on_shutdown = bool(self.get_parameter('save_plot_on_shutdown').value)
        self.plot_output_path = self.get_parameter('plot_output_path').value
        self.save_route_log_on_shutdown = bool(self.get_parameter('save_route_log_on_shutdown').value)
        self.route_log_output_path = self.get_parameter('route_log_output_path').value
        self.show_live_plot = bool(self.get_parameter('show_live_plot').value)
        self.undefined_object_margin = float(self.get_parameter('undefined_object_margin').value)
        self.save_simplified_route_on_shutdown = bool(
            self.get_parameter('save_simplified_route_on_shutdown').value
        )
        self.simplified_plot_output_path = self.get_parameter(
            'simplified_plot_output_path'
        ).value
        self.simplified_route_log_output_path = self.get_parameter(
            'simplified_route_log_output_path'
        ).value
        self.loop_closure_distance = float(
            self.get_parameter('loop_closure_distance').value
        )
        self.simplification_tolerance = float(
            self.get_parameter('simplification_tolerance').value
        )
        self.route_overlay_plot_output_path = self.get_parameter(
            'route_overlay_plot_output_path'
        ).value

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.latest_left_ticks = None
        self.latest_right_ticks = None
        self.latest_imu_yaw = None
        self.initial_imu_yaw = None
        self.tof_distance = None
        self.left_direction = 1
        self.right_direction = 1
        self.x_positions = [self.x]
        self.y_positions = [self.y]
        
        self.undefined_object_x = []
        self.undefined_object_y = []
        
        self.route_log = []

        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.frame_id

        self.path_pub = self.create_publisher(PathMsg, self.get_parameter('path_topic').value, 10)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 10)

        self.create_subscription(Int64, self.left_ticks_topic, self.left_ticks_cb, 10)
        self.create_subscription(Int64, self.right_ticks_topic, self.right_ticks_cb, 10)
        self.create_subscription(Int8, self.left_direction_topic, self.left_direction_cb, 10)
        self.create_subscription(Int8, self.right_direction_topic, self.right_direction_cb, 10)
        self.create_subscription(Float32, self.imu_yaw_topic, self.imu_yaw_cb, 10)
        
        # --- PATCH CONFIGURATION: Force compatibility matching profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(Range, self.tof_topic, self.tof_cb, qos_profile)
        
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.update_timer = self.create_timer(1.0 / update_rate_hz, self.update_route)

        self.figure = None
        self.axis = None
        self.line = None
        self.undefined_obj_line = None 
        
        if self.show_live_plot:
            self._init_live_plot()

        self.get_logger().info(
            'Route plotter ready. Target Environment: %s. Heading source: %s.'
            % ('JETSON (VNC)' if self.run_on_jetson else 'LAPTOP (Native)', self.heading_source)
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
        
    def tof_cb(self, msg):
        return
        # Read directly from the hardware msg layout
        self.tof_distance = float(msg.range)
        
        if self.tof_distance < 0.5:  
            cand_x = self.x - self.tof_distance * math.cos(self.theta)
            cand_y = self.y - self.tof_distance * math.sin(self.theta)
            
            already_flagged = False
            for ex_x, ex_y in zip(self.undefined_object_x, self.undefined_object_y):
                dist = math.hypot(cand_x - ex_x, cand_y - ex_y)
                if dist < self.undefined_object_margin:
                    already_flagged = True
                    break
            
            if not already_flagged:
                self.undefined_object_x.append(cand_x)
                self.undefined_object_y.append(cand_y)
                self.get_logger().info(
                    'New undefined object flagged at position: X=%.2f, Y=%.2f' % (cand_x, cand_y)
                )

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
            delta_left_ticks, delta_right_ticks, delta_left, delta_right, delta_theta, self.get_imu_theta(),
        )
        self.publish_pose()
        self.update_live_plot()

    def ticks_to_distance(self, ticks):
        return (ticks / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius

    def select_heading(self, encoder_theta):
        imu_theta = self.get_imu_theta()
        if self.heading_source == 'imu' and imu_theta is not None:
            return imu_theta
        if self.heading_source == 'fused' and imu_theta is not None:
            weight = min(max(self.imu_heading_weight, 0.0), 1.0)
            return self.blend_angles(encoder_theta, imu_theta, weight)
        return encoder_theta

    def get_imu_theta(self):
        if self.latest_imu_yaw is None or self.initial_imu_yaw is None:
            return None
        relative_imu_yaw = self.shortest_angle(self.initial_imu_yaw, self.latest_imu_yaw)
        return self.normalize_angle(float(self.get_parameter('initial_theta').value) + relative_imu_yaw)

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

    def append_route_log(self, delta_left_ticks, delta_right_ticks, delta_left, delta_right, delta_theta, imu_theta=None):
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
        global plt
        try:
            import matplotlib
            if self.run_on_jetson:
                matplotlib.use('TkAgg') 
            import matplotlib.pyplot as p
            plt = p
        except ImportError:
            self.get_logger().warning('matplotlib is not installed; live plotting disabled.')
            self.show_live_plot = False
            return

        plt.ion()
        self.figure, self.axis = plt.subplots()
        (self.line,) = self.axis.plot([], [], marker='o', label='Robot Path')
        (self.undefined_obj_line,) = self.axis.plot([], [], 'o', color='gray', markersize=4, label='Undefined Objects')
        
        self.axis.set_aspect('equal', adjustable='datalim')
        self.axis.set_xlabel('x position [m]')
        self.axis.set_ylabel('y position [m]')
        self.axis.set_title('Robot route from wheel encoders')
        self.axis.grid(True)
        self.axis.legend()
        
        plt.show(block=False)
        plt.pause(0.1)

    def update_live_plot(self):
        if not self.show_live_plot or self.figure is None or plt is None:
            return

        self.line.set_data(self.x_positions, self.y_positions)
        self.undefined_obj_line.set_data(self.undefined_object_x, self.undefined_object_y)
        self.axis.relim()
        self.axis.autoscale_view()
        
        if self.run_on_jetson:
            plt.pause(0.001)  
        else:
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()  

    def save_plot(self):
        if len(self.x_positions) < 2:
            self.get_logger().info('Route plot not saved because no movement was recorded.')
            return

        global plt
        if plt is None:
            try:
                import matplotlib.pyplot as p
                plt = p
            except ImportError:
                return

        output_path = Path(os.path.expanduser(self.plot_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        figure, axis = plt.subplots()
        axis.plot(self.x_positions, self.y_positions, marker='o', label='Robot Path')
        
        if self.undefined_object_x:
            axis.plot(self.undefined_object_x, self.undefined_object_y, 'o', color='gray', markersize=4, label='Undefined Objects')
            
        axis.set_aspect('equal', adjustable='datalim')
        axis.set_xlabel('x position [m]')
        axis.set_ylabel('y position [m]')
        axis.set_title('Robot route from wheel encoders')
        axis.grid(True)
        axis.legend()
        figure.savefig(str(self.timestamped_output_path(output_path)), bbox_inches='tight')
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

        fieldnames = ['time_sec', 'left_ticks', 'right_ticks', 'delta_left_ticks', 'delta_right_ticks', 'delta_left_m', 'delta_right_m', 'delta_theta_rad', 'imu_theta_rad', 'heading_source', 'x_m', 'y_m', 'theta_rad']
        with output_path.open('w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.route_log)

        self.get_logger().info('Saved route log to %s' % output_path)
    def get_simplified_route(self):
        points = list(zip(self.x_positions, self.y_positions))
        loop_erased = loop_erase_route(points, self.loop_closure_distance)
        return ramer_douglas_peucker(loop_erased, self.simplification_tolerance)

    def save_simplified_route(self):
        simplified_points = self.get_simplified_route()
        if len(simplified_points) < 2:
            self.get_logger().info(
                'Simplified route not saved because no movement was recorded.'
            )
            return

        self.save_simplified_route_plot(simplified_points)
        self.save_simplified_route_log(simplified_points)
        self.save_route_overlay_plot(simplified_points)

    def save_simplified_route_plot(self, simplified_points):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning(
                'matplotlib is not installed; simplified route image was not saved.'
            )
            return

        output_path = Path(os.path.expanduser(self.simplified_plot_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        x_values = [point[0] for point in simplified_points]
        y_values = [point[1] for point in simplified_points]

        figure, axis = plt.subplots()
        axis.plot(x_values, y_values, marker='o')
        axis.set_aspect('equal', adjustable='datalim')
        axis.set_xlabel('x position [m]')
        axis.set_ylabel('y position [m]')
        axis.set_title('Loop-erased route from start to finish')
        axis.grid(True)
        figure.savefig(str(self.timestamped_output_path(output_path)), bbox_inches='tight')
        plt.close(figure)
        self.get_logger().info('Saved simplified route plot to %s' % output_path)

    def save_simplified_route_log(self, simplified_points):
        import csv

        output_path = Path(os.path.expanduser(self.simplified_route_log_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open('w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=['point_index', 'x_m', 'y_m'])
            writer.writeheader()
            for index, point in enumerate(simplified_points):
                writer.writerow({
                    'point_index': index,
                    'x_m': point[0],
                    'y_m': point[1],
                })

        self.get_logger().info('Saved simplified route log to %s' % output_path)

    def save_route_overlay_plot(self, simplified_points):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning(
                'matplotlib is not installed; overlay route image was not saved.'
            )
            return

        output_path = Path(os.path.expanduser(self.route_overlay_plot_output_path))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        simple_x = [point[0] for point in simplified_points]
        simple_y = [point[1] for point in simplified_points]

        figure, axis = plt.subplots()
        axis.plot(
            self.x_positions,
            self.y_positions,
            marker='o',
            color='tab:blue',
            alpha=0.45,
            label='Raw route',
        )

        if self.undefined_object_x:
            axis.plot(
                self.undefined_object_x,
                self.undefined_object_y,
                'o',
                color='gray',
                markersize=4,
                label='Undefined objects',
            )

        axis.plot(
            simple_x,
            simple_y,
            '--',
            color='red',
            linewidth=2.5,
            marker='o',
            label='Simplified route',
        )
        axis.set_aspect('equal', adjustable='datalim')
        axis.set_xlabel('x position [m]')
        axis.set_ylabel('y position [m]')
        axis.set_title('Raw route with simplified route overlay')
        axis.grid(True)
        axis.legend()
        figure.savefig(str(self.timestamped_output_path(output_path)), bbox_inches='tight')
        plt.close(figure)
        self.get_logger().info('Saved route overlay plot to %s' % output_path)

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def shortest_angle(cls, from_angle, to_angle):
        return cls.normalize_angle(to_angle - from_angle)

    @staticmethod
    def timestamped_output_path(output_path):
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        return output_path.with_name(f"{now_str}_{output_path.name}")

    @classmethod
    def blend_angles(cls, encoder_theta, imu_theta, imu_weight):
        return cls.normalize_angle(encoder_theta + cls.shortest_angle(encoder_theta, imu_theta) * imu_weight)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.save_plot_on_shutdown:
            node.save_plot()
        if node.save_simplified_route_on_shutdown:
            node.save_simplified_route()
        if node.save_route_log_on_shutdown:
            node.save_route_log()
        if plt is not None:
            plt.close("all")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
    