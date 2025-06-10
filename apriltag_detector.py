import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag
from filterpy.kalman import KalmanFilter

class ApriltagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # AprilTag detector
        self.detector = apriltag.Detector()

        # Kalman filters per tag_id
        self.kalman_filters = {}

        # CV Bridge
        self.bridge = CvBridge()

        # Camera calibration (replace with your own)
        self.camera_matrix = np.array([
            [921.17, 0.0, 459.90],
            [0.0, 919.01, 351.23],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.033, 0.105, 0.001, -0.006, 0.0])

        # Tag size in meters
        self.tag_size = 0.05

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag/pose', 10)
        self.detection_pub = self.create_publisher(Int32MultiArray, '/apriltag/detections', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/apriltag/annotated_image', 10)

        # Subscriber
        self.image_sub = self.create_subscription(Image, '/tello/image', self.image_callback, 10)

        self.get_logger().info("✅ AprilTag Detector Node with Kalman Filter initialized")

    def create_kalman_filter(self):
        kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 0.1  # Assume ~10Hz update rate

        kf.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ])
        kf.P *= 10.0
        kf.R = np.eye(3) * 0.05  # measurement noise
        kf.Q = np.eye(6) * 0.01  # process noise

        return kf

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(gray)
            tag_ids = []

            for det in detections:
                tag_id = det.tag_id
                tag_ids.append(tag_id)

                corners = det.corners.reshape((4, 2)).astype(np.float32)

                # Define 3D object points
                half = self.tag_size / 2
                object_points = np.array([
                    [-half, -half, 0],
                    [ half, -half, 0],
                    [ half,  half, 0],
                    [-half,  half, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    corners,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    tvec = tvec.reshape(-1)

                    # Init or update Kalman filter
                    if tag_id not in self.kalman_filters:
                        kf = self.create_kalman_filter()
                        kf.x[:3] = tvec.reshape((3, 1))
                        self.kalman_filters[tag_id] = kf

                    kf = self.kalman_filters[tag_id]
                    kf.predict()
                    kf.update(tvec)
                    filtered = kf.x[:3].reshape(-1)

                    # Publish smoothed pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = f"apriltag_{tag_id}"
                    pose_msg.pose.position.x = float(filtered[0])
                    pose_msg.pose.position.y = float(filtered[1])
                    pose_msg.pose.position.z = float(filtered[2])
                    self.pose_pub.publish(pose_msg)

                # Draw box and ID
                corners = corners.astype(int)
                cv2.polylines(cv_image, [corners], True, (0, 255, 0), 2)
                cv2.putText(cv_image, str(tag_id), tuple(corners[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Publish detected IDs
            if tag_ids:
                ids_msg = Int32MultiArray()
                ids_msg.data = tag_ids
                self.detection_pub.publish(ids_msg)

            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f"[AprilTag] Detection Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AprilTag detector")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
