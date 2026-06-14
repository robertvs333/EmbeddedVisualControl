#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  
import cv2
from cv_bridge import CvBridge  
import numpy as np
  
class ImageSubscriber(Node):  
    def __init__(self):  
        super().__init__('image_processor')  
  
        # Subscribe to the raw camera stream using Best Effort
        self.sub = self.create_subscription(  
            CompressedImage,  
            '/camera/image_raw',  
            self.image_cb,
            qos_profile_sensor_data
        )

        # FIX: Changed '10' to 'qos_profile_sensor_data' so your laptop nodes can receive it!
        self.pub = self.create_publisher(
            CompressedImage, 
            '/camera/image_undistorted', 
            qos_profile_sensor_data
        )
  
        self.bridge = CvBridge()  
        self.latest_frame_undistorted = None
        self.initialized = True
        
        # --- CAMERA CALIBRATION MATRICES ---
        self.mtx = np.array([[414.0021984042558, 0.0, 327.5245129429971], [0.0, 549.8578103672768, 277.9068764271801], [0.0, 0.0, 1.0]])
        self.dist = np.array([[-0.38763539957385413, 0.2014454790072049, -0.003947477103130129, -0.0022718116503923055, -0.05779010370643695]])
        
        # --- INITIALIZATION LOG ---
        self.get_logger().info("Image processing node successfully initialized! Active on /camera/image_undistorted")
  
    def radial_lens_shading_calibration(self, channel_bgr):
        h, w = channel_bgr.shape[:2]
        xc, yc = w // 2, h // 2
        
        x = np.arange(w) - xc
        y = np.arange(h) - yc
        xx, yy = np.meshgrid(x, y)
        r2 = xx**2 + yy**2
        
        gain_matrix = 1.0 + 3.8e-6 * r2 - 1.2e-12 * (r2**2)
        
        corrected = channel_bgr.astype(np.float32)
        for i in range(3):
            corrected[:, :, i] *= gain_matrix
            
        return np.clip(corrected, 0, 255).astype(np.uint8)

    def image_cb(self, data):  
        try:  
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')  

            # FLIP OPERATION: Correct inverted sensor mounting alignments
            cv_image = cv2.flip(cv_image, -1)

            # CHROMATIC LENS FILTER: Strip the circular pink vignetting
            cv_image = self.radial_lens_shading_calibration(cv_image)

            # GEOMETRIC FLAT FILTER: Rescale lens projection warping
            self.latest_frame_undistorted = cv2.undistort(
                cv_image,
                self.mtx,
                self.dist
            )

            undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(self.latest_frame_undistorted)
            
            undistorted_msg.header.stamp = data.header.stamp
            undistorted_msg.header.frame_id = data.header.frame_id
            
            self.pub.publish(undistorted_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing/publishing: {e}")

            
def main(args=None):  
    rclpy.init(args=args)  
    node = ImageSubscriber()  
  
    try:  
        rclpy.spin(node)
    except KeyboardInterrupt:  
        pass  
    finally:  
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()