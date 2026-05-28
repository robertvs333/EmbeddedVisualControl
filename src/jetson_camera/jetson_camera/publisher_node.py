#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage  
import cv2  
from cv_bridge import CvBridge  
  
  
class CameraPublisher(Node):  
    def __init__(self):  
        super().__init__('camera_publisher')  
  
        # publisher for the image topic
        self.publisher_ = self.create_publisher(
                CompressedImage, 
                '/camera/image_raw', 
                qos_profile_sensor_data)  

        # cv bridge for converting images
        self.bridge = CvBridge()  
        
        # load parameters from .yaml configuration file
        config = self.load_parameters()

        self.sensor_id = config["sensor_id"]
        self.width = config["width"]
        self.height = config["height"]
        self.fps = config["fps"]
        
        self.gst_pipeline = self.define_gstreamer_pipeline()
        if self.gst_pipeline is None:
            self.get_logger().error('Invalid sensor id in GStreamer pipeline')  
  
        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)  
  
        if not self.cap.isOpened():  
            self.get_logger().error('Failed to open GStreamer pipeline')  
            raise RuntimeError('Camera open failed')  

        self.first_image_received = False
        self.initialized = True
        self.get_logger().info('Camera publisher node initialized!')

    def load_parameters(self):
            config = {}

            # declare parameters (otherwise ROS ignores the .yaml configuration file!)
            self.declare_parameter('sensor_id', 0)
            self.declare_parameter('width', 640)
            self.declare_parameter('height', 480)
            self.declare_parameter('fps', 10)

            # read parameters from configuration file
            config["sensor_id"] = self.get_parameter('sensor_id').value
            config["width"] = self.get_parameter('width').value
            config["height"] = self.get_parameter('height').value
            config["fps"] = self.get_parameter('fps').value

            self.get_logger().info(f'Loaded config {config}')

            return config

    def define_gstreamer_pipeline(self):
        if self.sensor_id == 0:
            return (f"nvarguscamerasrc sensor-id=0 ! "
                    f"video/x-raw(memory:NVMM),width={self.width},height={self.height},framerate={self.fps}/1,format=NV12 ! "
                    f"nvvidconv ! video/x-raw,format=BGRx ! "
                    f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false max-buffers=1")
        elif self.sensor_id == 1:
            return (f"v4l2src device=/dev/video1 ! "
                    f"video/x-raw,format=YUY2,width={self.width},height={self.height},framerate={self.fps}/1 ! "
                    f"videoconvert ! video/x-raw,format=BGR ! appsink sync=false max-buffers=1 drop=true")
        else:
            return None

    def pub_frame(self, frame):
        msg = self.bridge.cv2_to_compressed_imgmsg(frame)  
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)  
  
  
def main(args=None):  
    rclpy.init(args=args)  
    node = CameraPublisher()  
  
    try:  
        while rclpy.ok():  
            rclpy.spin_once(node, timeout_sec=0.0)  
            ret, frame = node.cap.read()  
            if not ret:  
                continue  

            node.pub_frame(frame)
  
  
    except KeyboardInterrupt:  
        pass  
    finally:  
        node.cap.release()  
        node.destroy_node()  
        rclpy.shutdown()  
  
  
if __name__ == '__main__':  
    main()
