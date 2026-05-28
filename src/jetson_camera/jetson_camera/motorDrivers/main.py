from jetson_camera.motorDrivers import *
from time import sleep

import math
import numpy as np
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver


class PositionController:
    def __init__(self, wheel_radius=0.033, axle_width=0.19, trim=0.0):
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

        # Linear calibration gains: maps [-1, +1] command -> rad/s
        # gain * command = physical wheel speed in rad/s
        # These are updated by calibrate()
        self.left_gain = 1.0   # rad/s per unit command
        self.right_gain = 1.0  # rad/s per unit command

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _measure_wheel_speed(self, left_cmd, right_cmd, duration=1.0):
        """
        Run both motors at the given command values for `duration` seconds,
        then return the measured angular speed (rad/s) for each wheel.
        Positive ticks = forward rotation.
        """
        # Reset tick snapshot
        t0_left = self.left_encoder._ticks
        t0_right = self.right_encoder._ticks

        self.motor.set_wheels_speed(left=left_cmd, right=right_cmd)
        sleep(duration)
        self.motor.set_wheels_speed(left=0.0, right=0.0)
        sleep(0.2)  # brief settle

        d_left  = self.left_encoder._ticks  - t0_left
        d_right = self.right_encoder._ticks - t0_right

        # ticks -> radians:  2*pi radians per resolution ticks
        rad_left  = 2 * math.pi * d_left  / self.resolution
        rad_right = 2 * math.pi * d_right / self.resolution

        omega_left  = rad_left  / duration  # rad/s
        omega_right = rad_right / duration  # rad/s

        return omega_left, omega_right

    def calibrate(self, sweep_steps=5, step_duration=10.0, verbose=True):
        """
        Sweep each motor independently through `sweep_steps` positive command
        values in (0, 1] and one negative value (-0.5) to cover both directions.

        For each sample the commanded value (x) and measured rad/s (y) are
        collected.  A least-squares linear fit through the origin
            y = gain * x
        gives a single gain per motor.

        The gains are stored in self.left_gain and self.right_gain.
        """
        commands = list(np.linspace(0.2, 1.0, sweep_steps)) + [-0.5]

        left_cmds,  left_omegas  = [], []
        right_cmds, right_omegas = [], []

        print("=== Starting motor calibration sweep ===")

        # --- Left motor sweep (right motor off) ---
        print("\n-- Left motor --")
        for cmd in commands:
            omega_l, _ = self._measure_wheel_speed(left_cmd=cmd, right_cmd=0.0,
                                                   duration=step_duration)
            left_cmds.append(cmd)
            left_omegas.append(omega_l)
            if verbose:
                print(f"  cmd={cmd:+.2f}  ->  {omega_l:+.4f} rad/s")

        # --- Right motor sweep (left motor off) ---
        print("\n-- Right motor --")
        for cmd in commands:
            _, omega_r = self._measure_wheel_speed(left_cmd=0.0, right_cmd=cmd,
                                                   duration=step_duration)
            right_cmds.append(cmd)
            right_omegas.append(omega_r)
            if verbose:
                print(f"  cmd={cmd:+.2f}  ->  {omega_r:+.4f} rad/s")

        # --- Least-squares linear fit through origin: gain = sum(x*y)/sum(x^2) ---
        lx = np.array(left_cmds)
        ly = np.array(left_omegas)
        rx = np.array(right_cmds)
        ry = np.array(right_omegas)

        self.left_gain  = float(np.dot(lx, ly) / np.dot(lx, lx))
        self.right_gain = float(np.dot(rx, ry) / np.dot(rx, rx))

        print("\n=== Calibration result ===")
        print(f"  Left  gain: {self.left_gain:.4f} rad/s per unit command")
        print(f"  Right gain: {self.right_gain:.4f} rad/s per unit command")
        print("==========================\n")

    def cmd_to_rad_per_s(self, left_cmd, right_cmd):
        """Convert a [-1,+1] command pair to physical rad/s using stored gains."""
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
            left=velocity - self.trim * velocity,
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
            self.motor.set_wheels_speed(
                #left=velocity  - self.trim * velocity,
                left=0,
                #right=0
                right=-velocity + self.trim * velocity
                )
        else:
            self.motor.set_wheels_speed(
                #left=-velocity + self.trim * velocity,
                left=0,
                #right=0
                right=velocity - self.trim * velocity
                )

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

velocity   = 0.2
Controller = PositionController(trim=0)

try:
    # Run calibration first; comment out if already calibrated
    #Controller.calibrate(sweep_steps=5, step_duration=5.0)

    # Example: print what 0.5 command means in physical units after calibration
    #l_rads, r_rads = Controller.cmd_to_rad_per_s(0.5, 0.5)
    #print(f"Command 0.5 -> Left: {l_rads:.4f} rad/s | Right: {r_rads:.4f} rad/s")

    #Controller.move_forward(1, velocity=-velocity)
    Controller.rotate(-math.pi / 2, velocity=velocity)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    Controller.close()