import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
import cv2
import numpy as np
from jetson_interfaces.msg import DualImage

# Motor driver imports (Assuming these are still needed elsewhere in your environment)
from jetson_camera.motorDrivers.motorDriver import *
from time import sleep
import math
from jetson_camera.motorDrivers.encoderDriver import WheelEncoderDriver


class FaceDetectionDisplay(Node):
    def __init__(self):
        super().__init__('face_detector_display')

        # Load the pre-trained face classifier (Standard OpenCV)
        self.face_cascade = cv2.CascadeClassifier(
           '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        )

        # Publisher for the calculated rotation angle (in radians)
        self.rotation_pub = self.create_publisher(Float32, '/face_tracking/rotation_angle', 10)

        self.sub = self.create_subscription(  
            DualImage,  
            '/camera/dual_images',  
            self.image_cb,
            qos_profile_sensor_data
        )  
        
        # Approximate horizontal Field of View (FOV) of the camera in radians.
        self.fov_x_rad = 1.047 
        
        self.get_logger().info("Face Detection Node started! Publishing rotation angles.")

    def image_cb(self, data):
        try:
            # 1. Decode Undistorted Image
            np_arr = np.frombuffer(data.undistorted_image.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 2. Convert to Grayscale
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            # 3. Detect Faces
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            # Image dimensions to find the center
            img_height, img_width, _ = cv_img.shape
            frame_center_x = img_width // 2

            if len(faces) > 0:
                # Track the largest face (usually the closest)
                largest_face = max(faces, key=lambda rect: rect[2] * rect[3])
                x, y, w, h = largest_face

                # Calculate horizontal center of the face
                face_center_x = x + (w // 2)
                
                # Calculate Pixel Error (Positive = face is to the right, Negative = left)
                pixel_error = face_center_x - frame_center_x
                
                # 4. Calculate Rotation Angle
                rotation_angle = (pixel_error / img_width) * self.fov_x_rad
                
                # Print the adjustment to the terminal
                print(f"Adjustment needed: {rotation_angle:.3f} rad")
                
                # 5. Publish the calculated angle
                msg = Float32()
                msg.data = float(rotation_angle)
                self.rotation_pub.publish(msg)

                # --- Visualizations ---
                # Only draw the boundary box around the tracked face
                text_y = y - 10 if y - 10 > 20 else y + 30
                cv2.putText(cv_img, f"Angle: {rotation_angle:.3f} rad", 
                            (x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.rectangle(cv_img, (x, y), (x+w, y+h), (0, 255, 0), 2)


            # 6. Display
            cv2.imshow('Jetson Face Detection', cv_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionDisplay()
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