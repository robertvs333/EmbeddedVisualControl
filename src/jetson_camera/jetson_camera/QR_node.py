#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from jetson_interfaces.msg import DualImage
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge

class BarcodeDetectorNode(Node):
    def __init__(self):
        super().__init__('barcode_detector')

        # Subscribe to the dual image topic
        # Using a depth of 10 for better compatibility, but keeping your original name
        self.sub = self.create_subscription(
            DualImage,
            '/camera/dual_images',
            self.process_callback,
            qos_profile_sensor_data
        )

        # Publisher: sends True when a QR code pattern or data is detected.
        self.detection_pub = self.create_publisher(Bool, '/detection', 10)

        self.bridge = CvBridge()

        # Initialize the QR detector
        self.qr_detector = cv2.QRCodeDetector()

        self.get_logger().info("Barcode/QR Detector Node started - Robust Version.")

    def process_callback(self, msg):
        try:
            # 1. Decode Image from the message
            # We use the undistorted_image field from your DualImage message
            np_arr = np.frombuffer(msg.undistorted_image.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("Received empty image frame.")
                return

            # 2. Combined Detection and Decoding
            # data: The string content (e.g., the URL)
            # points: The 4 corners of the QR code
            data, points, _ = self.qr_detector.detectAndDecode(frame)

            # 3. Robust "Found" Check
            # getattr(points, 'size', 0) safely handles if points is None or a Bool
            found_points = (points is not None and getattr(points, 'size', 0) > 0)
            found_data = (data is not None and len(data) > 0)

            if found_points or found_data:
                # --- ACTION: PUBLISH TO THE ROBOT ---
                # We use a standard print with flush=True to ensure it bypasses terminal buffering
                print(">>> QR DETECTED! PUBLISHING TRUE TO /DETECTION", flush=True)
                
                msg_out = Bool()
                msg_out.data = True
                self.detection_pub.publish(msg_out)
                
                # Log the data if available
                if found_data:
                    self.get_logger().info(f"QR Content: {data}")

                # 4. Draw Visual Feedback (Green Box)
                if found_points:
                    try:
                        pts = points.astype(int).reshape(-1, 2)
                        for i in range(len(pts)):
                            cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]), (0, 255, 0), 3)
                    except Exception as e_draw:
                        self.get_logger().warn(f"Drawing failed: {e_draw}")

            # --- DISPLAY THE RESULT ---
            # Note: If running headless (SSH), cv2.imshow may cause errors. 
            # Comment these out if the node hangs without a monitor.
            cv2.imshow("QR Detector Stream", frame)
            cv2.waitKey(1)

        except Exception as e:
            # Catch-all to prevent the node from dying if something goes wrong
            self.get_logger().error(f"DETECTION CRASHED: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BarcodeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()