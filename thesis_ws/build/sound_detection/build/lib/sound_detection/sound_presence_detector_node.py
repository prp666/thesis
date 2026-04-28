import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int16MultiArray, Bool, Float32


class SoundPresenceDetectorNode(Node):
    def __init__(self):
        super().__init__('sound_presence_detector_node')

        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/sound/audio',
            self.audio_callback,
            10
        )

        self.detect_pub = self.create_publisher(Bool, '/sound/detected', 10)
        self.conf_pub = self.create_publisher(Float32, '/sound/confidence', 10)

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('energy_threshold', 1e5)
        self.declare_parameter('ratio_threshold', 0.05)
        self.declare_parameter('confidence_alpha', 0.25)
        self.declare_parameter('detection_hold_sec', 0.5)
        self.declare_parameter('log_every_n_messages', 10)

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)
        self.ratio_threshold = float(self.get_parameter('ratio_threshold').value)
        self.confidence_alpha = float(self.get_parameter('confidence_alpha').value)
        self.detection_hold_sec = float(self.get_parameter('detection_hold_sec').value)
        self.log_every_n_messages = max(
            1, int(self.get_parameter('log_every_n_messages').value)
        )
        self.smoothed_confidence = 0.0
        self.last_positive_time = 0.0
        self.message_count = 0

        self.get_logger().info('SoundPresenceDetectorNode started.')

    def audio_callback(self, msg: Int16MultiArray):
        samples = np.array(msg.data, dtype=np.float32)

        if len(samples) == 0:
            return

        energy = float(np.mean(samples ** 2))

        fft_vals = np.fft.rfft(samples)
        freqs = np.fft.rfftfreq(len(samples), d=1.0 / self.sample_rate)
        power = np.abs(fft_vals) ** 2

        band_mask = (freqs >= 150.0) & (freqs <= 600.0)
        band_energy = float(np.sum(power[band_mask]))
        total_energy = float(np.sum(power) + 1e-8)
        band_ratio = float(band_energy / total_energy)

        raw_detected = bool(
            (energy > self.energy_threshold) and
            (band_ratio > self.ratio_threshold)
        )

        energy_ratio = float(energy / self.energy_threshold)
        band_ratio_norm = float(band_ratio / self.ratio_threshold)

        energy_score = np.log1p(max(0.0, energy_ratio - 1.0)) / np.log1p(10.0)
        band_score = np.log1p(max(0.0, band_ratio_norm - 1.0)) / np.log1p(10.0)

        if not raw_detected:
            confidence = 0.12 * max(energy_score, band_score)
        else:
            confidence = 0.4 + 0.3 * energy_score + 0.3 * band_score

        confidence = float(max(0.0, min(confidence, 1.0)))
        self.smoothed_confidence = (
            self.confidence_alpha * confidence +
            (1.0 - self.confidence_alpha) * self.smoothed_confidence
        )

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if raw_detected:
            self.last_positive_time = now_sec

        detected = raw_detected or (
            self.last_positive_time > 0.0 and
            (now_sec - self.last_positive_time) <= self.detection_hold_sec and
            self.smoothed_confidence >= 0.2
        )

        detect_msg = Bool()
        detect_msg.data = detected

        conf_msg = Float32()
        conf_msg.data = float(self.smoothed_confidence)

        self.detect_pub.publish(detect_msg)
        self.conf_pub.publish(conf_msg)

        self.message_count += 1
        if self.message_count % self.log_every_n_messages == 0 or detected != raw_detected:
            self.get_logger().info(
                f'energy={energy:.2f}, band_ratio={band_ratio:.3f}, '
                f'raw_detected={raw_detected}, detected={detected}, '
                f'confidence={self.smoothed_confidence:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SoundPresenceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
