#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int64, Float32MultiArray, Float32
import math

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        self.motor = DaguWheelsDriver()

        # --- ROTATION PD & TRACTION TUNING PARAMETERS ---
        self.MAX_ROT_PWR = 0.32
        self.MIN_ROT_PWR = 0.15

        # PD Controller Gains
        self.KP_ROT = 0.55
        self.KD_ROT = 0.08

        # Tolerances
        self.ROT_TOLERANCE  = math.radians(2.0)
        self.SETTLE_TIME    = 0.30
        self.INTER_TURN_DELAY = 0.1

        # Robot Physical Constants
        self.wheel_radius       = 0.0325
        self.axle_width         = 0.185
        self.encoder_resolution = 140.0
        self.RIGHT_TRIM         = 1.105

        # Linear Driving Constants — UNTOUCHED from your working version
        self.BASE_PWR      = 0.45
        self.MIN_STEER_PWR = 0.25
        self.MAX_PWR       = 0.6
        self.KP_STRAIGHT   = 2.0
        self.RAMP_DISTANCE = 0.15

        # State Variables
        self.state           = "IDLE"
        self.target_distance = 0.0
        self.target_degrees  = 0.0
        self.target_yaw      = 0.0

        self.left_ticks       = 0
        self.right_ticks      = 0
        self.current_yaw      = 0.0
        self.yaw_history      = []
        self.last_wheel_state = [0, 0]

        self.t1_start          = 0
        self.t2_start          = 0
        self.locked_yaw        = 0.0
        self.stop_counter      = 0
        self.settle_start_time = None
        self.pause_start_time  = None

        self.rotation_queue = []
        self.last_yaw_error = 0.0
        self.rot_pwr_cmd    = 0.0

        # --- STALL KICK RECOVERY PARAMETERS ---
        #
        # Key design decisions vs previous version:
        #
        # 1. STALL_KICK_PWR is SCALED by remaining error so it can't overshoot
        #    when already close to the target (was flat 0.60 regardless of error).
        #
        # 2. Inside every kick tick we check for SUCCESS and OVERSHOOT and exit
        #    the kick early — previously the kick ran blindly to STALL_KICK_TICKS
        #    even after crossing the target, which caused error to explode.
        #
        # 3. After a kick ends, PD resumes from POST_KICK_PWR (above the stall
        #    floor) instead of MIN_ROT_PWR, preventing an immediate re-stall.
        #
        # 4. Stall detection is suppressed when error is already small (< 5°)
        #    because near the target the PD naturally slows and a slow approach
        #    should not be treated as a stall.
        #
        self.STALL_WINDOW    = 15               # 0.3 s observation window
        self.STALL_THRESHOLD = math.radians(1.5)

        # Kick power scales linearly between these two values based on error size
        self.STALL_KICK_PWR_FAR  = 0.75        # Full power for errors > STALL_KICK_FAR_ANGLE
        self.STALL_KICK_PWR_NEAR = 0.42        # Reduced power for small remaining errors
        self.STALL_KICK_FAR_ANGLE  = math.radians(20.0)  # Above this → full kick power
        self.STALL_KICK_NEAR_ANGLE = math.radians(5.0)   # Below this → reduced kick power

        self.STALL_KICK_TICKS   = 35           # Max kick duration (0.7 s) — longer for large errors
        self.POST_KICK_PWR      = 0.28         # Resume PD from here after kick (above stall floor)
        self.MAX_STALL_RETRIES  = 4

        # Internal stall tracking — reset each segment
        self.stall_error_history = []
        self.stall_kick_ticks    = 0
        self.stall_kick_dir      = (0, 0)
        self.stall_kick_sign     = 0            # Sign of error when kick was triggered
        self.stall_retry_count   = 0
        self.stall_active        = False

        # IMU Watchdog
        self.last_yaw_time       = self.get_clock().now()
        self.imu_timeout_seconds = 0.15

        # Comms
        self.status_pub    = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub  = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)

        self.create_subscription(Float32MultiArray, '/cmd_movement', self.instruction_cb, 10)
        self.create_subscription(Int64, '/encoders/left_ticks', self.left_tick_cb, 20)
        self.create_subscription(Int64, '/encoders/right_ticks', self.right_tick_cb, 20)
        self.create_subscription(Float32, '/imu/yaw', self.yaw_cb, 20)

        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Unified Smooth Movement & Sequential Turning Node Active.")

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def left_tick_cb(self, msg):  self.left_ticks  = msg.data
    def right_tick_cb(self, msg): self.right_ticks = msg.data

    def yaw_cb(self, msg):
        self.last_yaw_time = self.get_clock().now()
        self.yaw_history.append(msg.data)
        if len(self.yaw_history) > 3:
            self.yaw_history.pop(0)
        sin_sum = sum(math.sin(y) for y in self.yaw_history)
        cos_sum = sum(math.cos(y) for y in self.yaw_history)
        self.current_yaw = math.atan2(sin_sum, cos_sum)

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _reset_stall_state(self):
        self.stall_error_history = []
        self.stall_kick_ticks    = 0
        self.stall_kick_dir      = (0, 0)
        self.stall_kick_sign     = 0
        self.stall_retry_count   = 0
        self.stall_active        = False

    def _kick_power_for_error(self, abs_error):
        """Scale kick power linearly between NEAR and FAR values based on error."""
        if abs_error >= self.STALL_KICK_FAR_ANGLE:
            return self.STALL_KICK_PWR_FAR
        elif abs_error <= self.STALL_KICK_NEAR_ANGLE:
            return self.STALL_KICK_PWR_NEAR
        else:
            t = (abs_error - self.STALL_KICK_NEAR_ANGLE) / (self.STALL_KICK_FAR_ANGLE - self.STALL_KICK_NEAR_ANGLE)
            return self.STALL_KICK_PWR_NEAR + t * (self.STALL_KICK_PWR_FAR - self.STALL_KICK_PWR_NEAR)

    # ------------------------------------------------------------------ #
    #  Instruction Callback                                                #
    # ------------------------------------------------------------------ #

    def instruction_cb(self, msg):
        if self.state != "IDLE" or len(msg.data) < 2:
            return
        dist, angle = msg.data[0], msg.data[1]
        if dist != 0.0:
            self.target_distance = dist
            self.state = "START_LIN"
        elif angle != 0.0:
            self.rotation_queue = []
            remaining_angle = angle
            while abs(remaining_angle) > 90.0:
                sign = 1.0 if remaining_angle > 0 else -1.0
                self.rotation_queue.append(sign * 90.0)
                remaining_angle -= sign * 90.0
            if remaining_angle != 0.0:
                self.rotation_queue.append(remaining_angle)
            if self.rotation_queue:
                self.target_degrees = self.rotation_queue.pop(0)
                self.state = "START_ROT"

    # ------------------------------------------------------------------ #
    #  Control Loop                                                        #
    # ------------------------------------------------------------------ #

    def control_loop(self):
        if self.state == "IDLE":
            return

        # ---- LINEAR INIT ------------------------------------------------
        elif self.state == "START_LIN":
            self.t1_start, self.t2_start = self.left_ticks, self.right_ticks
            self.locked_yaw = self.current_yaw
            self.state = "LIN"

        # ---- LINEAR DRIVE: COMPLETELY UNTOUCHED FROM YOUR WORKING VERSION ----
        elif self.state == "LIN":
            direction = -1 if self.target_distance > 0 else 1
            self.set_wheel_state(direction, direction)
            cur_l = abs(self.left_ticks - self.t1_start)
            cur_r = abs(self.right_ticks - self.t2_start)
            dist_traveled = ((cur_l + cur_r) / 2.0 / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            remaining = abs(self.target_distance) - dist_traveled

            if dist_traveled >= abs(self.target_distance):
                self.state = "STOPPING"
                return

            ramp_up_factor   = min(1.0, dist_traveled / self.RAMP_DISTANCE)
            ramp_down_factor = min(1.0, remaining     / self.RAMP_DISTANCE)
            current_base = self.MIN_STEER_PWR + (self.BASE_PWR - self.MIN_STEER_PWR) * min(ramp_up_factor, ramp_down_factor)

            now      = self.get_clock().now()
            yaw_age  = (now - self.last_yaw_time).nanoseconds * 1e-9
            yaw_diff = 0.0 if yaw_age > self.imu_timeout_seconds else self.normalize_angle(self.current_yaw - self.locked_yaw)

            if direction == -1:
                l_pwr = current_base - (yaw_diff * self.KP_STRAIGHT)
                r_pwr = current_base + (yaw_diff * self.KP_STRAIGHT)
            else:
                l_pwr = current_base + (yaw_diff * self.KP_STRAIGHT)
                r_pwr = current_base - (yaw_diff * self.KP_STRAIGHT)

            l_pwr = max(min(self.MAX_PWR, l_pwr), self.MIN_STEER_PWR)
            r_pwr = max(min(self.MAX_PWR, r_pwr * self.RIGHT_TRIM), self.MIN_STEER_PWR * self.RIGHT_TRIM)
            self.motor.set_wheels_speed(l_pwr * direction, r_pwr * direction)

        # ---- ROTATION INIT ----------------------------------------------
        elif self.state == "START_ROT":
            self.get_logger().info(
                f"Executing turn segment: {self.target_degrees}° "
                f"(Queue depth left: {len(self.rotation_queue)})"
            )
            target_rad      = -math.radians(self.target_degrees)
            self.target_yaw = self.normalize_angle(self.current_yaw + target_rad)
            self.settle_start_time = None
            self.last_yaw_error    = self.normalize_angle(self.target_yaw - self.current_yaw)
            self.rot_pwr_cmd       = self.MIN_ROT_PWR
            self._reset_stall_state()
            self.state = "ROT"

        # ---- ROTATION ACTIVE --------------------------------------------
        elif self.state == "ROT":
            now     = self.get_clock().now()
            yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9

            if yaw_age > self.imu_timeout_seconds:
                self.get_logger().warn("IMU timeout during rotation.")
                self.state = "STOPPING"
                return

            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            abs_error = abs(yaw_error)

            # ---- SUCCESS REGION ----------------------------------------
            if abs_error <= self.ROT_TOLERANCE:
                self.motor.set_wheels_speed(0.0, 0.0)
                self.set_wheel_state(0, 0)
                if self.settle_start_time is None:
                    self.settle_start_time = now
                elif (now - self.settle_start_time).nanoseconds * 1e-9 >= self.SETTLE_TIME:
                    if len(self.rotation_queue) > 0:
                        self.get_logger().info("Segment complete. Entering recovery pause...")
                        self.pause_start_time = now
                        self.state = "ROT_PAUSE"
                    else:
                        self.get_logger().info(
                            f"Full Sequence Complete. Final Error={math.degrees(yaw_error):.2f}°"
                        )
                        self.state = "STOPPING"
                return

            self.settle_start_time = None

            # =================================================================
            # STALL KICK — runs instead of normal PD while active.
            #
            # Each tick inside the kick we check three exit conditions:
            #   1. SUCCESS  — entered the tolerance band → hand off to settle logic
            #   2. OVERSHOOT — error sign flipped (robot crossed the target) →
            #                  exit immediately so PD can correct the other way
            #   3. TIMEOUT  — STALL_KICK_TICKS elapsed normally
            #
            # Kick power is scaled to the current error so a large kick doesn't
            # slam through the target when only a few degrees remain.
            # =================================================================
            if self.stall_active:
                # Re-evaluate error every kick tick
                kick_pwr = self._kick_power_for_error(abs_error)

                # --- EXIT: success ---
                if abs_error <= self.ROT_TOLERANCE:
                    self.get_logger().info(
                        f"[KICK {self.stall_retry_count}/{self.MAX_STALL_RETRIES}] "
                        f"Reached tolerance mid-kick. Handing off to settle."
                    )
                    self.stall_active = False
                    self.motor.set_wheels_speed(0.0, 0.0)
                    self.set_wheel_state(0, 0)
                    self.settle_start_time = now
                    return

                # --- EXIT: overshoot (error sign flipped relative to kick start) ---
                current_sign = 1 if yaw_error > 0 else -1
                if current_sign != self.stall_kick_sign:
                    self.get_logger().info(
                        f"[KICK {self.stall_retry_count}/{self.MAX_STALL_RETRIES}] "
                        f"Overshoot detected at err={math.degrees(abs_error):.2f}°. "
                        f"Exiting kick early → PD resumes."
                    )
                    self.stall_active   = False
                    self.rot_pwr_cmd    = self.POST_KICK_PWR
                    self.last_yaw_error = yaw_error
                    self.stall_error_history.clear()
                    # Fall through to normal PD this same tick
                else:
                    # --- CONTINUE KICK ---
                    if self.stall_kick_ticks > 0:
                        self.stall_kick_ticks -= 1
                        self.set_wheel_state(self.stall_kick_dir[0], self.stall_kick_dir[1])
                        self.motor.set_wheels_speed(
                            kick_pwr * self.stall_kick_dir[0],
                            kick_pwr * self.RIGHT_TRIM * self.stall_kick_dir[1]
                        )
                        self.get_logger().info(
                            f"[KICK {self.stall_retry_count}/{self.MAX_STALL_RETRIES}] "
                            f"tick={self.STALL_KICK_TICKS - self.stall_kick_ticks}/{self.STALL_KICK_TICKS}  "
                            f"pwr={kick_pwr:.2f}  err={math.degrees(abs_error):.2f}°"
                        )
                        return  # Skip normal PD this tick

                    # --- EXIT: timeout ---
                    self.get_logger().info(
                        f"[KICK {self.stall_retry_count}/{self.MAX_STALL_RETRIES}] "
                        f"Timeout. Resuming PD from pwr={self.POST_KICK_PWR:.2f}."
                    )
                    self.stall_active   = False
                    self.rot_pwr_cmd    = self.POST_KICK_PWR   # don't drop back to MIN_ROT_PWR
                    self.last_yaw_error = yaw_error
                    self.stall_error_history.clear()
                    # Fall through to normal PD this same tick

            # ---- NORMAL PD PATH ----------------------------------------
            dt         = 0.02
            derivative = (yaw_error - self.last_yaw_error) / dt
            self.last_yaw_error = yaw_error

            pd_signal    = (self.KP_ROT * yaw_error) + (self.KD_ROT * derivative)
            target_power = max(self.MIN_ROT_PWR, min(self.MAX_ROT_PWR, abs(pd_signal)))

            if self.rot_pwr_cmd < target_power:
                self.rot_pwr_cmd = min(target_power, self.rot_pwr_cmd + 0.012)
            else:
                self.rot_pwr_cmd = max(target_power, self.rot_pwr_cmd - 0.025)

            # ---- STALL DETECTION ---------------------------------------
            # Only accumulate when error is large enough that slowness is
            # genuinely a stall — near the target the PD intentionally slows.
            STALL_MIN_ERROR = math.radians(5.0)
            if abs_error > STALL_MIN_ERROR:
                self.stall_error_history.append(abs_error)
                if len(self.stall_error_history) > self.STALL_WINDOW:
                    self.stall_error_history.pop(0)

                if len(self.stall_error_history) == self.STALL_WINDOW:
                    net_progress = abs(self.stall_error_history[0] - self.stall_error_history[-1])
                    if net_progress < self.STALL_THRESHOLD:
                        self.stall_retry_count += 1
                        if self.stall_retry_count > self.MAX_STALL_RETRIES:
                            self.get_logger().error(
                                f"Stall unrecoverable after {self.MAX_STALL_RETRIES} kicks "
                                f"(error={math.degrees(abs_error):.2f}°). Aborting."
                            )
                            self.state = "STOPPING"
                            return

                        if abs_error > math.radians(172.0):
                            kick_dir = (1, -1) if self.target_degrees > 0 else (-1, 1)
                        else:
                            kick_dir = (-1, 1) if pd_signal > 0 else (1, -1)

                        self.get_logger().warn(
                            f"STALL detected (progress={math.degrees(net_progress):.2f}° "
                            f"in {self.STALL_WINDOW} ticks, err={math.degrees(abs_error):.2f}°). "
                            f"Kick {self.stall_retry_count}/{self.MAX_STALL_RETRIES} "
                            f"at pwr={self._kick_power_for_error(abs_error):.2f}"
                        )
                        self.stall_active     = True
                        self.stall_kick_dir   = kick_dir
                        self.stall_kick_sign  = 1 if yaw_error > 0 else -1
                        self.stall_kick_ticks = self.STALL_KICK_TICKS
                        self.stall_error_history.clear()
                        return  # first kick tick handled next loop
            else:
                # Error is small — clear history so old stall data doesn't
                # carry over and fire a spurious kick near the target
                self.stall_error_history.clear()

            # ---- DIRECTION & 180° SAFEGUARD ----------------------------
            if abs_error > math.radians(172.0):
                left_dir, right_dir = (1, -1) if self.target_degrees > 0 else (-1, 1)
            else:
                left_dir, right_dir = (-1, 1) if pd_signal > 0 else (1, -1)

            self.set_wheel_state(left_dir, right_dir)
            self.motor.set_wheels_speed(
                self.rot_pwr_cmd * left_dir,
                self.rot_pwr_cmd * self.RIGHT_TRIM * right_dir
            )

        # ---- INTER-TURN PAUSE ------------------------------------------
        elif self.state == "ROT_PAUSE":
            self.motor.set_wheels_speed(0.0, 0.0)
            self.set_wheel_state(0, 0)
            now           = self.get_clock().now()
            elapsed_pause = (now - self.pause_start_time).nanoseconds * 1e-9
            if elapsed_pause >= self.INTER_TURN_DELAY:
                if len(self.rotation_queue) > 0:
                    self.target_degrees = self.rotation_queue.pop(0)
                    self.state = "START_ROT"
                else:
                    self.state = "STOPPING"

        # ---- STOPPING --------------------------------------------------
        elif self.state == "STOPPING":
            self.motor.set_wheels_speed(0.0, 0.0)
            self.set_wheel_state(0, 0)
            self.stop_counter += 1
            if self.stop_counter >= 10:
                self.stop_counter = 0
                self.status_pub.publish(Bool(data=True))
                self.state = "IDLE"

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self.motor.set_wheels_speed(0.0, 0.0)
        self.motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
