#!/usr/bin/env python3

import ast
import csv
import json
import math
import os
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String


CELL_UNKNOWN = 0
CELL_ROBOT = 1
CELL_UNIDENTIFIED_OBJECT = 2
CELL_NON_PERSON_OBJECT = 3
CELL_PERSON = 4

plt = None


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GridMappingNode(Node):
    """Build a simple semantic 2D grid map from route pose, ToF, and vision result."""

    def __init__(self):
        super().__init__('grid_mapping_node')

        self.declare_parameter('cell_size_m', 0.05)
        self.declare_parameter('room_width_m', 5.0)
        self.declare_parameter('room_height_m', 5.0)
        self.declare_parameter('start_in_center', True)
        self.declare_parameter('origin_x_m', 0.0)
        self.declare_parameter('origin_y_m', 0.0)
        self.declare_parameter('odom_topic', '/route/odom')
        self.declare_parameter('tof_topic', '/tof/distance')
        self.declare_parameter('detection_result_topic', '/detection/final_result')
        self.declare_parameter('grid_topic', '/mapping/semantic_grid')
        self.declare_parameter('tof_detection_threshold_m', 0.5)
        self.declare_parameter('duplicate_object_distance_m', 0.10)
        self.declare_parameter('sensor_forward_sign', -1.0)
        self.declare_parameter('mark_raw_tof_as_unidentified', False)
        self.declare_parameter('detection_angle_sign', 1.0)
        self.declare_parameter('save_grid_on_shutdown', True)
        self.declare_parameter('grid_csv_output_path', 'semantic_grid.csv')
        self.declare_parameter('grid_plot_output_path', 'semantic_grid.png')
        self.declare_parameter('show_live_grid', True)
        self.declare_parameter('run_on_jetson', True)

        self.cell_size_m = float(self.get_parameter('cell_size_m').value)
        self.room_width_m = float(self.get_parameter('room_width_m').value)
        self.room_height_m = float(self.get_parameter('room_height_m').value)
        self.width_cells = int(math.ceil(self.room_width_m / self.cell_size_m))
        self.height_cells = int(math.ceil(self.room_height_m / self.cell_size_m))

        if bool(self.get_parameter('start_in_center').value):
            self.origin_x_m = -self.room_width_m * 0.5
            self.origin_y_m = -self.room_height_m * 0.5
        else:
            self.origin_x_m = float(self.get_parameter('origin_x_m').value)
            self.origin_y_m = float(self.get_parameter('origin_y_m').value)

        self.tof_detection_threshold_m = float(
            self.get_parameter('tof_detection_threshold_m').value
        )
        self.duplicate_object_distance_m = float(
            self.get_parameter('duplicate_object_distance_m').value
        )
        self.sensor_forward_sign = float(self.get_parameter('sensor_forward_sign').value)
        self.mark_raw_tof_as_unidentified = bool(
            self.get_parameter('mark_raw_tof_as_unidentified').value
        )
        self.detection_angle_sign = float(
            self.get_parameter('detection_angle_sign').value
        )
        self.save_grid_on_shutdown = bool(
            self.get_parameter('save_grid_on_shutdown').value
        )
        self.grid_csv_output_path = self.get_parameter('grid_csv_output_path').value
        self.grid_plot_output_path = self.get_parameter('grid_plot_output_path').value
        self.show_live_grid = bool(self.get_parameter('show_live_grid').value)
        self.run_on_jetson = bool(self.get_parameter('run_on_jetson').value)

        self.grid = [
            [CELL_UNKNOWN for _ in range(self.width_cells)]
            for _ in range(self.height_cells)
        ]
        self.robot_cell = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.latest_tof_distance = None
        self.objects = []
        self.figure = None
        self.axis = None
        self.image = None
        self.cmap = None
        self.norm = None

        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('grid_topic').value,
            10,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_cb,
            10,
        )

        tof_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(
            Range,
            self.get_parameter('tof_topic').value,
            self.tof_cb,
            tof_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('detection_result_topic').value,
            self.detection_result_cb,
            10,
        )

        if self.show_live_grid:
            self.init_live_grid()

        self.publish_timer = self.create_timer(0.5, self.publish_grid)
        self.get_logger().info(
            'Grid mapper ready: %dx%d cells, %.2f m/cell'
            % (self.width_cells, self.height_cells, self.cell_size_m)
        )

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = yaw_from_quaternion(msg.pose.pose.orientation)

        cell = self.world_to_cell(self.robot_x, self.robot_y)
        if cell is not None:
            self.robot_cell = cell
        else:
            self.get_logger().warn(
                'Robot pose outside grid: x=%.2f y=%.2f'
                % (self.robot_x, self.robot_y),
                throttle_duration_sec=2.0,
            )

    def tof_cb(self, msg):
        distance = float(msg.range)
        self.latest_tof_distance = distance

        if (
            self.mark_raw_tof_as_unidentified
            and 0.0 < distance <= self.tof_detection_threshold_m
        ):
            self.mark_object_from_distance(distance, CELL_UNIDENTIFIED_OBJECT)

    def detection_result_cb(self, msg):
        payload = self.parse_detection_payload(msg.data)
        if not payload:
            return

        if self.handle_multi_object_payload(payload):
            return

        target_type = str(
            payload.get('_type', payload.get('type', payload.get('class', 'none')))
        ).lower()
        distance = payload.get('distance_m', self.latest_tof_distance)
        relative_angle = self.get_payload_angle(payload)
        if distance is None:
            return

        try:
            distance = float(distance)
        except (TypeError, ValueError):
            return

        if distance <= 0.0:
            return

        if target_type in ('person', 'face', 'human'):
            cell_value = CELL_PERSON
        elif target_type == 'object':
            cell_value = CELL_NON_PERSON_OBJECT
        elif target_type in ('unknown', 'unidentified'):
            cell_value = CELL_UNIDENTIFIED_OBJECT
        else:
            return

        self.mark_object_from_distance(distance, cell_value, relative_angle)

    def handle_multi_object_payload(self, payload):
        angles = payload.get('final_averaged_angles')
        if not angles:
            return False

        distance = payload.get('distance_m', self.latest_tof_distance)
        if distance is None:
            return True

        try:
            distance = float(distance)
        except (TypeError, ValueError):
            return True

        if distance <= 0.0:
            return True

        for angle in angles:
            try:
                relative_angle = float(angle)
            except (TypeError, ValueError):
                continue
            self.mark_object_from_distance(
                distance,
                CELL_UNIDENTIFIED_OBJECT,
                relative_angle,
            )
        return True

    def get_payload_angle(self, payload):
        for key in ('angle_rad', 'relative_angle_rad', 'tracking_angle'):
            if key in payload:
                try:
                    return float(payload[key])
                except (TypeError, ValueError):
                    return 0.0

        if 'angle_deg' in payload:
            try:
                return math.radians(float(payload['angle_deg']))
            except (TypeError, ValueError):
                return 0.0

        return 0.0

    def mark_object_from_distance(self, distance, cell_value, relative_angle=0.0):
        base_theta = self.robot_theta
        if self.sensor_forward_sign < 0.0:
            base_theta += math.pi

        bearing = base_theta + self.detection_angle_sign * relative_angle
        object_x = self.robot_x + distance * math.cos(bearing)
        object_y = self.robot_y + distance * math.sin(bearing)
        cell = self.world_to_cell(object_x, object_y)

        if cell is None:
            self.get_logger().warn(
                'Detected object outside grid: x=%.2f y=%.2f'
                % (object_x, object_y),
                throttle_duration_sec=2.0,
            )
            return

        existing_index = self.find_nearby_object(object_x, object_y)
        if existing_index is not None:
            old = self.objects[existing_index]
            upgraded_value = max(old['value'], cell_value)
            old.update({'x': object_x, 'y': object_y, 'cell': cell, 'value': upgraded_value})
            self.set_cell(cell, upgraded_value)
            return

        self.objects.append({
            'x': object_x,
            'y': object_y,
            'cell': cell,
            'value': cell_value,
        })
        self.set_cell(cell, cell_value)
        self.get_logger().info(
            'Mapped cell %s as %d at x=%.2f y=%.2f'
            % (cell, cell_value, object_x, object_y)
        )

    def find_nearby_object(self, x, y):
        for index, obj in enumerate(self.objects):
            if math.hypot(x - obj['x'], y - obj['y']) <= self.duplicate_object_distance_m:
                return index
        return None

    def set_cell(self, cell, value):
        col, row = cell
        self.grid[row][col] = value

    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x_m) / self.cell_size_m))
        row = int(math.floor((y - self.origin_y_m) / self.cell_size_m))

        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            return col, row
        return None

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.cell_size_m
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x_m
        msg.info.origin.position.y = self.origin_y_m
        msg.info.origin.orientation.w = 1.0

        display_grid = [row[:] for row in self.grid]
        if self.robot_cell is not None:
            col, row = self.robot_cell
            display_grid[row][col] = CELL_ROBOT

        msg.data = [
            display_grid[row][col]
            for row in range(self.height_cells)
            for col in range(self.width_cells)
        ]
        self.grid_pub.publish(msg)
        self.update_live_grid(display_grid)

    def init_live_grid(self):
        global plt
        try:
            import matplotlib
            if self.run_on_jetson:
                matplotlib.use('TkAgg')
            import matplotlib.pyplot as p
            from matplotlib.colors import BoundaryNorm, ListedColormap
        except ImportError:
            self.get_logger().warning(
                'matplotlib is not installed; live grid display disabled.'
            )
            self.show_live_grid = False
            return

        plt = p
        self.cmap = ListedColormap(['white', 'dodgerblue', 'gray', 'orange', 'red'])
        self.norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], self.cmap.N)

        plt.ion()
        self.figure, self.axis = plt.subplots()
        self.image = self.axis.imshow(
            self.get_display_grid(),
            origin='lower',
            cmap=self.cmap,
            norm=self.norm,
            interpolation='nearest',
        )
        self.axis.set_title('Semantic Grid Map')
        self.axis.set_xlabel('grid x')
        self.axis.set_ylabel('grid y')
        self.axis.grid(color='black', linewidth=0.2)

        legend_items = [
            ('0 undefined', 'white'),
            ('1 robot', 'dodgerblue'),
            ('2 unidentified', 'gray'),
            ('3 not person', 'orange'),
            ('4 person', 'red'),
        ]
        handles = [
            plt.Line2D([0], [0], marker='s', color='black',
                       markerfacecolor=color, linestyle='', label=label)
            for label, color in legend_items
        ]
        self.axis.legend(handles=handles, loc='upper right', fontsize='small')

        plt.show(block=False)
        plt.pause(0.1)

    def get_display_grid(self):
        display_grid = [row[:] for row in self.grid]
        if self.robot_cell is not None:
            col, row = self.robot_cell
            display_grid[row][col] = CELL_ROBOT
        return display_grid

    def update_live_grid(self, display_grid):
        if not self.show_live_grid or self.figure is None or plt is None:
            return

        self.image.set_data(display_grid)
        if self.run_on_jetson:
            plt.pause(0.001)
        else:
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()

    def save_grid(self):
        self.save_grid_csv()
        self.save_grid_plot()

    def save_grid_csv(self):
        output_path = self.resolve_output_path(self.grid_csv_output_path)
        display_grid = self.get_display_grid()

        with output_path.open('w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'cell_size_m',
                self.cell_size_m,
                'origin_x_m',
                self.origin_x_m,
                'origin_y_m',
                self.origin_y_m,
            ])
            writer.writerows(reversed(display_grid))

        self.get_logger().info('Saved semantic grid CSV to %s' % output_path)

    def save_grid_plot(self):
        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import ListedColormap, BoundaryNorm
        except ImportError:
            self.get_logger().warning('matplotlib is not installed; grid plot not saved.')
            return

        output_path = self.resolve_output_path(self.grid_plot_output_path)
        display_grid = self.get_display_grid()

        colors = ['white', 'dodgerblue', 'gray', 'orange', 'red']
        cmap = ListedColormap(colors)
        norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], cmap.N)

        figure, axis = plt.subplots()
        axis.imshow(display_grid, origin='lower', cmap=cmap, norm=norm)
        axis.set_title('Semantic Grid Map')
        axis.set_xlabel('grid x')
        axis.set_ylabel('grid y')
        axis.grid(color='black', linewidth=0.2)
        figure.savefig(str(output_path), bbox_inches='tight')
        plt.close(figure)
        self.get_logger().info('Saved semantic grid plot to %s' % output_path)

    def resolve_output_path(self, path_value):
        output_path = Path(os.path.expanduser(path_value))
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        return output_path

    @staticmethod
    def parse_detection_payload(raw_payload):
        try:
            return json.loads(raw_payload)
        except json.JSONDecodeError:
            try:
                return ast.literal_eval(raw_payload)
            except (SyntaxError, ValueError):
                return None


def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.save_grid_on_shutdown:
            node.save_grid()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
