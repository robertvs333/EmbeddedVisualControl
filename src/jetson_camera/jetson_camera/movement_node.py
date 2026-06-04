#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Int8, Int64, Float32MultiArray
import math
import time

from jetson_camera.motorDrivers.motorDriver import DaguWheelsDriver

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        
        # 1. Use a Reentrant Callback Group to allow callbacks to run in parallel
        self.callback_group = ReentrantCallbackGroup()
        
        self.motor = DaguWheelsDriver()
        self.wheel_radius = 0.0325
        self.axle_width = 0.19
        self.encoder_resolution = 147.0
        
        self.SAMPLETIME = 0.1 
        self.TARGET = 20       
        self.KP = 0.004        
        self.KD = 0.0025       
        self.KI = 0.0005       

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_wheel_state = [0, 0]
        self.is_busy = False 

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/movement_finished', 10)
        self.left_dir_pub = self.create_publisher(Int8, '/motors/left_direction', 10)
        self.right_dir_pub = self.create_publisher(Int8, '/motors/right_direction', 10)
        
        # 2. Assign all subscribers to the reentrant callback group
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 
            '/cmd_movement', 
            self.instruction_cb, 
            10,
            callback_group=self.callback_group
        )
        self.left_tick_sub = self.create_subscription(
            Int64, 
            '/encoders/left_ticks', 
            self.left_tick_cb, 
            10,
            callback_group=self.callback_group
        )
        self.right_tick_sub = self.create_subscription(
            Int64, 
            '/encoders/right_ticks', 
            self.right_tick_cb, 
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("Movement Node (Multi-Threaded) ready.")

    def left_tick_cb(self, msg):
        self.left_ticks = msg.data

    def right_tick_cb(self, msg):
        self.right_ticks = msg.data

    def set_wheel_state(self, left, right):
        if [left, right] != self.last_wheel_state:
            l_msg, r_msg = Int8(), Int8()
            l_msg.data, r_msg.data = int(left), int(right)
            self.left_dir_pub.publish(l_msg)
            self.right_dir_pub.publish(r_msg)
            self.last_wheel_state = [left, right]
            time.sleep(0.05) 

    def instruction_cb(self, msg):
        if self.is_busy or len(msg.data) < 2:
            return
            
        dist, angle = msg.data[0], msg.data[1]
        self.is_busy = True
        
        try:
            if dist != 0.0:
                self.execute_linear_move(dist)
            elif angle != 0.0:
                self.execute_rotation(angle)
        finally:
            self.stop_and_finish()

    def execute_linear_move(self, distance):
        direction = -1 if distance > 0 else 1 
        self.set_wheel_state(direction, direction)

        m1_speed, m2_speed = 0.1, 0.1
        e1_prev_err, e2_prev_err = 0, 0
        e1_sum_err, e2_sum_err = 0, 0
        total_dist = 0.0
        
        # Start reference points
        t1_abs_start = self.left_ticks
        t2_abs_start = self.right_ticks

        while rclpy.ok() and abs(total_dist) < abs(distance):
            t1_sample_start = self.left_ticks
            t2_sample_start = self.right_ticks

            time.sleep(self.SAMPLETIME)

            # Because of the MultiThreadedExecutor, self.left_ticks is 
            # now updating in the background!
            e1_val = abs(self.left_ticks - t1_sample_start)
            e2_val = abs(self.right_ticks - t2_sample_start)

            e1_err = self.TARGET - e1_val
            e2_err = self.TARGET - e2_val

            m1_adj = (e1_err * self.KP) + ((e1_err - e1_prev_err) * self.KD) + (e1_sum_err * self.KI)
            m2_adj = (e2_err * self.KP) + ((e2_err - e2_prev_err) * self.KD) + (e2_sum_err * self.KI)

            m1_speed = max(min(0.8, m1_speed + m1_adj), 0.1)
            m2_speed = max(min(0.8, m2_speed + m2_adj), 0.1)

            self.motor.set_wheels_speed(m1_speed * direction, m2_speed * direction)
            
            e1_prev_err, e2_prev_err = e1_err, e2_err
            e1_sum_err = max(min(e1_sum_err + e1_err, 50), -50)
            e2_sum_err = max(min(e2_sum_err + e2_err, 50), -50)

            l_moved = abs(self.left_ticks - t1_abs_start)
            r_moved = abs(self.right_ticks - t2_abs_start)
            avg_ticks = (l_moved + r_moved) / 2.0
            total_dist = (avg_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius

    def execute_rotation(self, degrees, speed=0.3):
        target_rad = abs(degrees) * (math.pi / 180.0)
        
        if degrees > 0: 
            self.set_wheel_state(1, -1)
            self.motor.set_wheels_speed(speed, -speed)
        else: 
            self.set_wheel_state(-1, 1)
            self.motor.set_wheels_speed(-speed, speed)

        t1_abs_start = self.left_ticks
        t2_abs_start = self.right_ticks

        while rclpy.ok():
            l_dist = (abs(self.left_ticks - t1_abs_start) / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            r_dist = (abs(self.right_ticks - t2_abs_start) / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            
            if abs((r_dist + l_dist) / self.axle_width) >= target_rad:
                break
            time.sleep(0.01)

    def stop_and_finish(self):
        self.motor.set_wheels_speed(0.0, 0.0)
        self.set_wheel_state(0, 0)
        self.status_pub.publish(Bool(data=True))
        self.is_busy = False

    def destroy_node(self):
        self.motor.set_wheels_speed(0.0, 0.0)
        self.motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    
    # 3. Use MultiThreadedExecutor to allow the tick callbacks to fire 
    # while the instruction loop is running.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()