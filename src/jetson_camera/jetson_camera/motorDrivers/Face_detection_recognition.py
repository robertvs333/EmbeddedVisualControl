#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray
from insightface.app import FaceAnalysis

import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import json
import face_recognition

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.bridge = CvBridge()

        # 1. Configuration Parameters
        self.faces_dir = "/home/roel/vision_ws/known_faces"
        self.camera_fov_rad = 1.05
        self.sample_size = 20
        self.remembered_identity = "Unknown"
        self.is_scanning = False
        self.recognised = False
        self.state_history = []
        self.frame_count = 0
        self.detect_interval = 10

        self.tracker = None
        self.tracking = False
        self.current_bbox = None


        self.face_app = FaceAnalysis(
            name='buffalo_l',
            providers=['CPUExecutionProvider']
        )

        self.face_app.prepare(
            ctx_id=1,
            det_size=(640, 640)
        )

        # 2. Database Initialization
        self.known_face_encodings = []
        self.known_face_names = []
        self._load_face_database()

        # 3. ROS 2 Communication
        self.trigger_sub = self.create_subscription(String, '/detection/face_trigger', self.trigger_callback, 10)
        self.result_pub = self.create_publisher(String, '/detection/face_result', 10)
        self.angles_pub = self.create_publisher(Float32MultiArray, '/detection/face_tracking_angles', 10)
        self.image_sub = self.create_subscription(CompressedImage, '/camera/image_undistorted', self.image_callback, qos_profile=qos_profile_sensor_data)

        cv2.namedWindow("Face Recognition Diagnostic Desk", cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Node Operational. Last known: {self.remembered_identity}")

    def _load_face_database(self):
        if not os.path.exists(self.faces_dir):
            os.makedirs(self.faces_dir)
            return
        for filename in os.listdir(self.faces_dir):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                name = os.path.splitext(filename)[0]
                path = os.path.join(self.faces_dir, filename)
                try:
                    image = face_recognition.load_image_file(path)
                    encodings = face_recognition.face_encodings(image)
                    if encodings:
                        self.known_face_encodings.append(encodings[0])
                        self.known_face_names.append(name)
                        self.get_logger().info(f"Loaded: {name}")
                except Exception as e:
                    self.get_logger().error(f"Failed to process {filename}: {e}")

    def trigger_callback(self, msg):
        self.get_logger().info("Scan sequence triggered!")

        self.state_history = []
        self.is_scanning = True

        self.recognised = False
        self.remembered_identity = "Unknown"

        self.tracking = False
        self.tracker = None

    def image_callback(self, image_msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        if not self.is_scanning:

            cv2.putText(
                cv_img,
                "STANDBY",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        else:

            img_rgb = cv2.cvtColor(
                cv_img,
                cv2.COLOR_BGR2RGB
            )

            ih, iw = cv_img.shape[:2]

            self.frame_count += 1

            face_locs = []

            # Only run CNN occasionally
            need_detection = (
                not self.tracking
                or self.frame_count % self.detect_interval == 0
            )

            # --------------------------------------------------
            # CNN DETECTION
            # --------------------------------------------------
            if need_detection:

                faces = self.face_app.get(cv_img)

                for face in faces:

                    x1, y1, x2, y2 = face.bbox.astype(int)

                    face_locs.append((y1, x2, y2, x1))

                if face_locs:

                    # pick largest face
                    face_locs.sort(
                        key=lambda b: (b[2]-b[0]) * (b[1]-b[3]),
                        reverse=True
                    )

                    top, right, bottom, left = face_locs[0]

                    tracker_bbox = (
                        left,
                        top,
                        right - left,
                        bottom - top
                    )

                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(cv_img, tracker_bbox)

                    self.tracking = True

                    self.tracking = True

                    # ------------------------------------------
                    # Recognition only once
                    # ------------------------------------------
                    if (
                        not self.recognised
                        and self.known_face_encodings
                    ):

                        encs = face_recognition.face_encodings(
                            img_rgb,
                            known_face_locations=[
                                (top, right, bottom, left)
                            ]
                        )

                        if encs:

                            dists = face_recognition.face_distance(
                                self.known_face_encodings,
                                encs[0]
                            )

                            best_idx = np.argmin(dists)

                            if dists[best_idx] < 0.65:

                                self.remembered_identity = (
                                    self.known_face_names[best_idx]
                                )

                                self.recognised = True

            # --------------------------------------------------
            # TRACKING
            # --------------------------------------------------
            else:

                success, bbox = self.tracker.update(cv_img)

                if success:

                    x, y, w, h = [
                        int(v) for v in bbox
                    ]

                    left = x
                    top = y
                    right = x + w
                    bottom = y + h

                    face_locs = [
                        (top, right, bottom, left)
                    ]

                else:

                    self.tracking = False
                    self.tracker = None

            # --------------------------------------------------
            # DRAW + ANGLES
            # --------------------------------------------------
            for (top, right, bottom, left) in face_locs:

                cv2.rectangle(
                    cv_img,
                    (left, top),
                    (right, bottom),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    cv_img,
                    self.remembered_identity,
                    (left, top - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                face_center_x = (
                    left + (right - left) / 2
                )

                target_angle = (
                    0.5 - (face_center_x / iw)
                ) * self.camera_fov_rad

                msg = Float32MultiArray()
                msg.data = [target_angle]
                self.angles_pub.publish(msg)

            self.state_history.append(
                self.remembered_identity
            )

            if len(self.state_history) >= self.sample_size:
                self.evaluate_and_publish_final_result()

        cv2.imshow(
            "Face Recognition Diagnostic Desk",
            cv_img
        )

        cv2.waitKey(1)

    def evaluate_and_publish_final_result(self):
        self.is_scanning = False
        output = {"status": "complete", "identity": self.remembered_identity}
        self.result_pub.publish(String(data=json.dumps(output)))
        self.get_logger().info(f"Published: {output}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FaceRecognitionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()