import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int16MultiArray


class FakeAudioSourceNode(Node):
    def __init__(self):
        super().__init__('fake_audio_source_node')

        self.publisher_ = self.create_publisher(Int16MultiArray, '/sound/audio', 10)

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_size', 1024)
        self.declare_parameter('drone_present', True)
        self.declare_parameter('noise_level', 120.0)

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.frame_size = int(self.get_parameter('frame_size').value)
        self.noise_level = float(self.get_parameter('noise_level').value)

        self.sample_index = 0

        period = self.frame_size / self.sample_rate
        self.timer = self.create_timer(period, self.publish_audio)

        self.get_logger().info('FakeAudioSourceNode started.')

    def publish_audio(self):
        drone_present = bool(self.get_parameter('drone_present').value)

        n = np.arange(self.frame_size)
        t = (self.sample_index + n) / self.sample_rate

        noise = np.random.normal(0, self.noise_level, self.frame_size)

        if drone_present:
            f0 = 180.0
            f1 = 360.0
            f2 = 540.0

            envelope = 1.0 + 0.25 * np.sin(2 * np.pi * 3.0 * t)

            signal = (
                7000.0 * np.sin(2 * np.pi * f0 * t) +
                3500.0 * np.sin(2 * np.pi * f1 * t) +
                1800.0 * np.sin(2 * np.pi * f2 * t)
            )

            samples = envelope * signal + noise
        else:
            samples = noise

        self.sample_index += self.frame_size

        samples = np.clip(samples, -32768, 32767).astype(np.int16)

        msg = Int16MultiArray()
        msg.data = samples.tolist()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeAudioSourceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
