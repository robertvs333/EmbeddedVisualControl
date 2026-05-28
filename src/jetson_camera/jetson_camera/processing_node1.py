#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  
import cv2
from cv_bridge import CvBridge  
from rclpy.time import Time
import numpy as np
from jetson_interfaces.msg import DualImage
  
  
class ImageSubscriber(Node):  
    def __init__(self):  
        super().__init__('image_processor')  
  
        self.sub = self.create_subscription(  
            CompressedImage,  
            '/camera/image_raw',  
            self.image_cb,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(DualImage, '/camera/dual_images', 10)

  
        self.bridge = CvBridge()  
        self.latest_frame_distorted = None  
        self.latest_frame_undistorted = None
        self.first_image_received = False
        self.initialized = True
        self.mtx=np.array([[480.83813424,0.,284.55328677],
                  [0.,651.46808412,292.7034479],
                  [0.,0.,1.]])
        self.dist=np.array([[-5.88139283e-01,8.62139297e-01,8.41421702e-04,2.41098592e-02,-8.49124807e-01]])
  
    def image_cb(self, data):
        if not self.initialized:
            return

        try:
            # 1. Decode incoming compressed image to OpenCV (NumPy)
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = cv2.flip(cv_image,0)
            self.latest_frame_distorted = cv_image

            # 2. Undistort the image
            self.latest_frame_undistorted = cv2.undistort(
                self.latest_frame_distorted,
                self.mtx,
                self.dist
            )

            # 3. Create the Custom Message
            msg = DualImage()

            # OPTIMIZATION: Reuse the incoming raw message directly 
            # (No need to re-encode what is already encoded!)
            msg.raw_image = data 

            # 4. Convert the Undistorted NumPy array back to a ROS CompressedImage
            undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(self.latest_frame_undistorted)
            
            # Keep the original timestamp so we can track latency later
            undistorted_msg.header.stamp = data.header.stamp
            
            msg.undistorted_image = undistorted_msg

            # 5. Publish
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing/publishing: {e}")

            
def main(args=None):  
    rclpy.init(args=args)  
    node = ImageSubscriber()  
  
    try:  
        while rclpy.ok():  
            rclpy.spin_once(node, timeout_sec=0.01)  
  
            if node.latest_frame_undistorted is not None:  
                #cv2.imshow('Undistorted', node.latest_frame_undistorted)  
                #cv2.imshow('Distorted', node.latest_frame_distorted)
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
