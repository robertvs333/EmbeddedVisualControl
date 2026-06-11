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
  
        # Subscribe to the raw camera stream
        self.sub = self.create_subscription(  
            CompressedImage,  
            '/camera/image_raw',  
            self.image_cb,
            qos_profile_sensor_data
        )

        # Publish the flipped, color-corrected, undistorted output
        self.pub = self.create_publisher(CompressedImage, '/camera/image_undistorted', 10)
  
        self.bridge = CvBridge()  
        self.latest_frame_undistorted = None
        self.initialized = True
        
        # --- CAMERA CALIBRATION MATRICES ---
        self.mtx = np.array([[414.0021984042558, 0.0, 327.5245129429971], [0.0, 549.8578103672768, 277.9068764271801], [0.0, 0.0, 1.0]])
        self.dist = np.array([[-0.38763535261736637, 0.1206987338635489, 0.007239427505367979, -0.0012038137821843342, 0.2568056253712985]])
  
    def radial_lens_shading_calibration(self, img):
        """
        Generates a radial grid calculation to mathematically reduce and
        eliminate concentric pink hotspots and ring artifacts.
        """
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Create a coordinate plane relative to the lens center
        X, Y = np.meshgrid(np.arange(w), np.arange(h))
        
        # 2. Find pixel distance from center and normalize it (0.0 center to 1.0 corners)
        distance = np.sqrt((X - cx)**2 + (Y - cy)**2)
        max_dist = np.sqrt(cx**2 + cy**2)
        norm_dist = distance / max_dist

        # 3. Separate channels for targeting
        r, g, b = cv2.split(img)

        # 4. TUNING COEFFICIENTS:
        # Increase center_suppression if the pink dominates the inner image.
        # Increase edge_suppression if the pink shifts into an outer perimeter loop.
        center_suppression = 0.01 
        edge_suppression = 0.14    

        # 5. Build standard correction masks for Red and Blue profiles
        r_correction = 1.0 - (center_suppression * (1.0 - norm_dist) + edge_suppression * norm_dist)
        b_correction = 1.0 - (center_suppression * (1.0 - norm_dist) + edge_suppression * norm_dist)

        # 6. Apply spatial balancing and flatten limits
        r = np.clip(r * r_correction, 0, 255).astype(np.uint8)
        b = np.clip(b * b_correction, 0, 255).astype(np.uint8)

        return cv2.merge((r, g, b))

    def image_cb(self, data):
        if not self.initialized:
            return

        try:
            # 1. Decompress data stream into an array
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 2. ORIENTATION FIX: Perform full 180-degree flip (Vertical + Horizontal)
            cv_image = cv2.flip(cv_image, -1)

            # 3. COLOR CORRECTION: Swap raw sensor channels to true RGB formatting
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 4. CHROMATIC LENS FILTER: Strip the circular pink vignetting
            cv_image = self.radial_lens_shading_calibration(cv_image)

            # 5. GEOMETRIC FLAT FILTER: Rescale lens projection warping
            self.latest_frame_undistorted = cv2.undistort(
                cv_image,
                self.mtx,
                self.dist
            )

            # 6. Repack clean image structure back into a ROS CompressedImage message
            undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(self.latest_frame_undistorted)
            
            # Sync timing definitions with origin frames
            undistorted_msg.header.stamp = data.header.stamp
            undistorted_msg.header.frame_id = data.header.frame_id
            
            # 7. Broadcast the clear topic channel
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