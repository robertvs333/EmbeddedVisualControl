#!/usr/bin/env python3

from jetson_camera.motorDrivers.motorDriver import *
from time import sleep

import math
import threading
import numpy as np
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PositionController(Node):
    def __init__(self, wheel_radius=0.033, axle_width=0.19, trim=0.0):
        super().__init__('position_controller')

        self.motor = DaguWheelsDriver()
        self.wheel_radius = wheel_radius
        self.wheel_base = axle_width
        self.trim = trim
        self.resolution = 140.0
        self.left_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_1)
        self.right_encoder = WheelEncoderDriver(self.motor.GPIO_MOTOR_ENCODER_2)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.left_gain = 1.0
        self.right_gain = 1.0

        # Guard: while True the robot is mid-turn; new /detection messages are ignored
        self._is_turning = False

        # Subscribe to the QR detection topic
        self.create_subscription(Bool, '/detection', self._detection_callback, 10)
        self.get_logger().info("PositionController node started, waiting for /detection ...")

    # ------------------------------------------------------------------
    # Detection callback
    # ------------------------------------------------------------------

    def _detection_callback(self, msg: Bool):
        if not msg.data:
            return

        if self._is_turning:
            self.get_logger().info("Detection received but already turning — ignored.")
            return

        self.get_logger().info("QR detected — starting 90° turn.")
        # Run the blocking rotate in a background thread so rclpy.spin() stays responsive
        t = threading.Thread(target=self._turn_90, daemon=True)
        t.start()

    def _turn_90(self):
        """Blocking 90-degree turn executed in a background thread."""
        self._is_turning = True
        try:
            self.rotate(-math.pi / 2, velocity=0.2)
            self.get_logger().info("90° turn complete.")
        finally:
            self._is_turning = False

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _measure_wheel_speed(self, left_cmd, right_cmd, duration=1.0):
        t0_left  = self.left_encoder._ticks
        t0_right = self.right_encoder._ticks

        self.motor.set_wheels_speed(left=left_cmd, right=right_cmd)
        sleep(duration)
        self.motor.set_wheels_speed(left=0.0, right=0.0)
        sleep(0.2)

        d_left  = self.left_encoder._ticks  - t0_left
        d_right = self.right_encoder._ticks - t0_right

        rad_left  = 2 * math.pi * d_left  / self.resolution
        rad_right = 2 * math.pi * d_right / self.resolution

        return rad_left / duration, rad_right / duration

    def calibrate(self, sweep_steps=5, step_duration=10.0, verbose=True):
        commands = list(np.linspace(0.2, 1.0, sweep_steps)) + [-0.5]
        left_cmds,  left_omegas  = [], []
        right_cmds, right_omegas = [], []

        print("=== Starting motor calibration sweep ===")

        print("\n-- Left motor --")
        for cmd in commands:
            omega_l, _ = self._measure_wheel_speed(left_cmd=cmd, right_cmd=0.0,
                                                    duration=step_duration)
            left_cmds.append(cmd)
            left_omegas.append(omega_l)
            if verbose:
                print(f"  cmd={cmd:+.2f}  ->  {omega_l:+.4f} rad/s")

        print("\n-- Right motor --")
        for cmd in commands:
            _, omega_r = self._measure_wheel_speed(left_cmd=0.0, right_cmd=cmd,
                                                    duration=step_duration)
            right_cmds.append(cmd)
            right_omegas.append(omega_r)
            if verbose:
                print(f"  cmd={cmd:+.2f}  ->  {omega_r:+.4f} rad/s")

        lx = np.array(left_cmds);  ly = np.array(left_omegas)
        rx = np.array(right_cmds); ry = np.array(right_omegas)

        self.left_gain  = float(np.dot(lx, ly) / np.dot(lx, lx))
        self.right_gain = float(np.dot(rx, ry) / np.dot(rx, rx))

        print(f"\n=== Calibration result ===")
        print(f"  Left  gain: {self.left_gain:.4f} rad/s per unit command")
        print(f"  Right gain: {self.right_gain:.4f} rad/s per unit command")
        print("==========================\n")

    def cmd_to_rad_per_s(self, left_cmd, right_cmd):
        return self.left_gain * left_cmd, self.right_gain * right_cmd

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------

    def update_odometry(self):
        left_ticks  = self.left_encoder._ticks
        right_ticks = self.right_encoder._ticks

        d_left_ticks  = left_ticks  - self.prev_left_ticks
        d_right_ticks = right_ticks - self.prev_right_ticks

        self.prev_left_ticks  = left_ticks
        self.prev_right_ticks = right_ticks

        dist_left  = 2 * math.pi * self.wheel_radius * (d_left_ticks  / self.resolution)
        dist_right = 2 * math.pi * self.wheel_radius * (d_right_ticks / self.resolution)

        d_center = (dist_left + dist_right) / 2.0
        d_theta  = (dist_right - dist_left) / self.wheel_base

        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)
        self.theta += d_theta
        print("Theta: {:.4f} \t X: {:.4f} \t Y: {:.4f}".format(
            self.theta, self.x, self.y))

    def update_theta(self):
        left_ticks  = self.left_encoder._ticks
        right_ticks = self.right_encoder._ticks

        d_left_ticks  = left_ticks  - self.prev_left_ticks
        d_right_ticks = right_ticks - self.prev_right_ticks

        self.prev_left_ticks  = left_ticks
        self.prev_right_ticks = right_ticks

        dist_left  = 2 * math.pi * self.wheel_radius * (d_left_ticks  / self.resolution)
        dist_right = 2 * math.pi * self.wheel_radius * (d_right_ticks / self.resolution)

        d_theta = (dist_left + dist_right) / self.wheel_base
        self.theta += d_theta
        print("Theta: {:.4f}".format(self.theta))

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def move_forward(self, distance, velocity=0.5):
        start_x = self.x
        start_y = self.y
        self.motor.set_wheels_speed(
            left=velocity  - self.trim * velocity,
            right=velocity + self.trim * velocity)

        while True:
            self.update_odometry()
            travelled = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            if travelled >= distance:
                self.motor.set_wheels_speed(left=0.0, right=0.0)
                break
            sleep(0.01)

    def rotate(self, angle, velocity=0.5):
        start_theta = self.theta
        if angle > 0:
            self.motor.set_wheels_speed(left=0, right=-velocity + self.trim * velocity)
        else:
            self.motor.set_wheels_speed(left=0, right=velocity - self.trim * velocity)

        while True:
            self.update_theta()
            rotated = abs(self.theta - start_theta)
            if rotated >= abs(angle):
                self.motor.set_wheels_speed(left=0.0, right=0.0)
                break
            sleep(0.01)

    def go_to_pose(self, target_x, target_y, target_theta,
                   kp_rho=1.0, kp_alpha=2.0, kp_beta=-0.5):
        dx = target_x - self.x
        dy = target_y - self.y

        rho   = math.sqrt(dx**2 + dy**2)
        alpha = math.atan2(dy, dx) - self.theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        beta  = target_theta - self.theta - alpha
        beta  = (beta + math.pi) % (2 * math.pi) - math.pi

        v     = kp_rho  * rho
        omega = kp_alpha * alpha + kp_beta * beta

        v_left  = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        self.motor.set_wheels_speed(left=v_left, right=v_right)
        return rho < 0.05

    def close(self):
        self.motor.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    controller = PositionController(trim=0)
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()