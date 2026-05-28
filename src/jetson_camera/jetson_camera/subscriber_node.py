#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  
from jetson_interfaces.msg import DualImage
import cv2
from cv_bridge import CvBridge  
from rclpy.time import Time
import numpy as np
  
  
class ImageSubscriber(Node):  
    def __init__(self):  
        super().__init__('image_subscriber')  
  
        self.sub = self.create_subscription(  
            DualImage,  
            '/camera/dual_images',  
            self.image_cb,
            qos_profile_sensor_data
        )  
  
        self.bridge = CvBridge()  
        self.latest_frame_raw = None  
        self.latest_frame_undistorted = None
        self.first_image_received = False
        self.initialized = True
  
    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            self.get_logger().info(
                "Camera subscriber captured first image from publisher."
            )

        try:
            # compute PubSub delay
            msg_time = Time.from_msg(data.raw_image.header.stamp)
            now = self.get_clock().now()
            delay_sec = (now - msg_time).nanoseconds * 1e-9

            self.get_logger().info(f"PubSub delay: {delay_sec:.4f} s")

            # decode compressedimage, without CVBridge
            np_arr_raw = np.frombuffer(data.raw_image.data, np.uint8)
            np_arr_undistorted = np.frombuffer(data.undistorted_image.data, np.uint8)
            cv_image_raw = cv2.imdecode(np_arr_raw, cv2.IMREAD_COLOR)
            cv_image_undistorted = cv2.imdecode(np_arr_undistorted, cv2.IMREAD_COLOR)
            self.latest_frame_raw = cv_image_raw
            self.latest_frame_undistorted = cv_image_undistorted

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
 
  
def main(args=None):  
    rclpy.init(args=args)  
    node = ImageSubscriber()  
  
    try:  
        while rclpy.ok():  
            rclpy.spin_once(node, timeout_sec=0.01)  
  
            if node.latest_frame_raw is not None:  
                cv2.imshow('Raw Image', node.latest_frame_raw)  
            if node.latest_frame_undistorted is not None:  
                cv2.imshow('Undistorted Image', node.latest_frame_undistorted)  
            if cv2.waitKey(1) == 27:  # ESC  
                break  
  
    except KeyboardInterrupt:  
        pass  
    finally:  
        cv2.destroyAllWindows()  
        node.destroy_node()  
        rclpy.shutdown()  
  
  
if __name__ == '__main__':  
    main()
