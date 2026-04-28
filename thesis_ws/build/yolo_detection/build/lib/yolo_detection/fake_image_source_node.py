import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_IMAGE_DIR = '/home/prp/thesis/thesis_ws/src/yolo_detection/pictures'


class FakeImageSourceNode(Node):
    def __init__(self):
        super().__init__('fake_image_source_node')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('frame_id', 'fake_camera')
        self.declare_parameter('image_dir', DEFAULT_IMAGE_DIR)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('drone_present', True)
        self.declare_parameter('noise_level', 8.0)

        image_topic = str(self.get_parameter('image_topic').value)
        publish_rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.image_dir = str(self.get_parameter('image_dir').value)
        self.width = max(64, int(self.get_parameter('width').value))
        self.height = max(64, int(self.get_parameter('height').value))
        self.noise_level = max(0.0, float(self.get_parameter('noise_level').value))
        self.package_share_dir = get_package_share_directory('yolo_detection')
        workspace_root = os.path.dirname(
            os.path.dirname(os.path.dirname(self.package_share_dir))
        )
        self.package_source_dir = os.path.join(workspace_root, 'src', 'yolo_detection')

        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.frame_index = 0
        self.image_files = []
        self.image_file_index = 0
        self.refresh_image_files()

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)

        mode = 'image directory' if self.image_files else 'synthetic'
        self.get_logger().info(
            f'FakeImageSourceNode started. topic={image_topic}, mode={mode}, '
            f'image_dir={self.image_dir or "<none>"}'
        )
        if not self.image_files:
            self.get_logger().warn(
                'No valid images found in image_dir. Publishing synthetic frames for pipeline testing.'
            )

    def resolve_image_dir(self, image_dir: str) -> str:
        if not image_dir:
            return ''

        if os.path.isabs(image_dir) and os.path.isdir(image_dir):
            return image_dir

        candidates = [
            image_dir,
            os.path.join(self.package_source_dir, image_dir),
            os.path.join(self.package_share_dir, image_dir),
            os.path.join(self.package_source_dir, 'pictures'),
            os.path.join(self.package_share_dir, 'pictures'),
        ]

        for candidate in candidates:
            if os.path.isdir(candidate):
                return candidate

        return image_dir

    def refresh_image_files(self) -> None:
        resolved_dir = self.resolve_image_dir(self.image_dir)
        if not os.path.isdir(resolved_dir):
            self.image_files = []
            self.get_logger().warn(f'image_dir does not exist: {self.image_dir}')
            return

        valid_exts = ('.jpg', '.jpeg', '.png', '.bmp')
        self.image_files = sorted([
            os.path.join(resolved_dir, name)
            for name in os.listdir(resolved_dir)
            if name.lower().endswith(valid_exts)
        ])
        self.image_file_index = 0

    def load_current_image(self):
        if not self.image_files:
            return None

        image_path = self.image_files[self.image_file_index % len(self.image_files)]
        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().warn(f'Failed to read image file: {image_path}')
            return None

        self.image_file_index += 1
        return cv2.resize(image, (self.width, self.height))

    def generate_synthetic_frame(self, drone_present: bool) -> np.ndarray:
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        frame[:] = (210, 235, 255)
        horizon = int(self.height * 0.72)
        frame[horizon:, :] = (80, 160, 80)

        sun_x = int(self.width * 0.15 + 12 * np.sin(self.frame_index * 0.15))
        cv2.circle(frame, (sun_x, int(self.height * 0.18)), 26, (0, 220, 255), -1)

        if drone_present:
            cx = int(self.width * 0.5 + 40 * np.sin(self.frame_index * 0.2))
            cy = int(self.height * 0.35 + 25 * np.cos(self.frame_index * 0.12))
            arm = 34
            rotor = 14

            cv2.line(frame, (cx - arm, cy - arm), (cx + arm, cy + arm), (40, 40, 40), 4)
            cv2.line(frame, (cx - arm, cy + arm), (cx + arm, cy - arm), (40, 40, 40), 4)
            cv2.rectangle(frame, (cx - 10, cy - 8), (cx + 10, cy + 8), (30, 30, 30), -1)
            for rx, ry in [
                (cx - arm, cy - arm),
                (cx + arm, cy + arm),
                (cx - arm, cy + arm),
                (cx + arm, cy - arm),
            ]:
                cv2.circle(frame, (rx, ry), rotor, (55, 55, 55), 2)

            cv2.putText(
                frame,
                'synthetic drone',
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                'no drone',
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )

        if self.noise_level > 0.0:
            noise = np.random.normal(0.0, self.noise_level, frame.shape)
            frame = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        return frame

    def build_frame(self) -> np.ndarray:
        drone_present = bool(self.get_parameter('drone_present').value)
        updated_image_dir = str(self.get_parameter('image_dir').value)

        if updated_image_dir != self.image_dir:
            self.image_dir = updated_image_dir
            self.refresh_image_files()

        frame = self.load_current_image()
        if frame is not None:
            return frame
        else:
            return self.generate_synthetic_frame(drone_present)

    def publish_image(self) -> None:
        frame = self.build_frame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)
        self.frame_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeImageSourceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
