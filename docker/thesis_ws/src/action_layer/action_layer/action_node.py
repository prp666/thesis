import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')

        self.declare_parameter('spoof_triggered_topic', '/decision/spoof_triggered')
        self.declare_parameter('decision_state_topic', '/decision/state')
        self.declare_parameter('decision_reason_topic', '/decision/reason')
        self.declare_parameter('action_active_topic', '/action/active')
        self.declare_parameter('action_status_topic', '/action/status')
        self.declare_parameter('auto_clear_states', ['IDLE', 'MONITORING'])
        self.declare_parameter('status_publish_period_sec', 0.5)

        spoof_triggered_topic = str(self.get_parameter('spoof_triggered_topic').value)
        decision_state_topic = str(self.get_parameter('decision_state_topic').value)
        decision_reason_topic = str(self.get_parameter('decision_reason_topic').value)
        action_active_topic = str(self.get_parameter('action_active_topic').value)
        action_status_topic = str(self.get_parameter('action_status_topic').value)

        self.auto_clear_states = set(self.get_parameter('auto_clear_states').value)
        publish_period = max(
            0.1,
            float(self.get_parameter('status_publish_period_sec').value),
        )

        self.active = False
        self.last_state = 'UNKNOWN'
        self.last_reason = ''
        self.last_trigger_time = 0.0

        self.active_pub = self.create_publisher(Bool, action_active_topic, 10)
        self.status_pub = self.create_publisher(String, action_status_topic, 10)

        self.create_subscription(
            Bool,
            spoof_triggered_topic,
            self.spoof_triggered_callback,
            10,
        )
        self.create_subscription(
            String,
            decision_state_topic,
            self.decision_state_callback,
            10,
        )
        self.create_subscription(
            String,
            decision_reason_topic,
            self.decision_reason_callback,
            10,
        )

        self.timer = self.create_timer(publish_period, self.publish_status)

        self.get_logger().info(
            'ActionNode started without Gazebo/QGroundControl integration. '
            f'trigger_topic={spoof_triggered_topic}, state_topic={decision_state_topic}'
        )

    def spoof_triggered_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return

        self.active = True
        self.last_trigger_time = time.monotonic()
        self.get_logger().warn('Action triggered by decision layer.')

    def decision_state_callback(self, msg: String) -> None:
        self.last_state = msg.data
        if self.active and self.last_state in self.auto_clear_states:
            self.active = False
            self.get_logger().info(
                f'Action cleared because decision state became {self.last_state}.'
            )

    def decision_reason_callback(self, msg: String) -> None:
        self.last_reason = msg.data

    def publish_status(self) -> None:
        active_msg = Bool()
        active_msg.data = self.active
        self.active_pub.publish(active_msg)

        status_msg = String()
        status_msg.data = (
            f'active={self.active}, '
            f'last_state={self.last_state}, '
            f'last_trigger_age_sec={self.trigger_age_sec():.2f}, '
            f'last_reason="{self.last_reason}"'
        )
        self.status_pub.publish(status_msg)

    def trigger_age_sec(self) -> float:
        if self.last_trigger_time <= 0.0:
            return -1.0
        return time.monotonic() - self.last_trigger_time


def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
