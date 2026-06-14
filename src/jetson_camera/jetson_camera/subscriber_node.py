#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  # Changed from DualImage
import cv2
from cv_bridge import CvBridge  
from rclpy.time import Time
import numpy as np
  
class ImageSubscriber(Node):  
    def __init__(self):  
        super().__init__('image_subscriber')  
  
        # FIX: Listen to the actual topic and type coming from processing_node
        self.sub = self.create_subscription(  
            CompressedImage,  
            '/camera/image_undistorted',  
            self.image_cb,
            qos_profile_sensor_data
        )  
  
        self.bridge = CvBridge()  
        self.latest_frame_undistorted = None # Streamlined to look at the processed feed
        self.first_image_received = False
        self.initialized = True
  
    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            self.get_logger().info(
                "Camera subscriber captured first image from processing node!"
            )

        try:
            # Compute PubSub network latency delay
            msg_time = Time.from_msg(data.header.stamp)
            now = self.get_clock().now()
            delay_sec = (now - msg_time).nanoseconds * 1e-9

            self.get_logger().info(f"PubSub Latency: {delay_sec:.4f} s")

            # Decode the incoming compressed image array
            np_arr_undistorted = np.frombuffer(data.data, np.uint8)
            cv_image_undistorted = cv2.imdecode(np_arr_undistorted, cv2.IMREAD_COLOR)
            
            self.latest_frame_undistorted = cv_image_undistorted

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
 
  
def main(args=None):  
    rclpy.init(args=args)  
    node = ImageSubscriber()  
  
    try:  
        while rclpy.ok():  
            rclpy.spin_once(node, timeout_sec=0.01)  
  
            # Display window when a valid frame arrives
            if node.latest_frame_undistorted is not None:  
                cv2.imshow('Undistorted Image Feed', node.latest_frame_undistorted)  
            
            if cv2.waitKey(1) == 27:  # Press ESC to exit cleanly  
                break  
  
    except KeyboardInterrupt:  
        pass  
    finally:  
        cv2.destroyAllWindows()  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()