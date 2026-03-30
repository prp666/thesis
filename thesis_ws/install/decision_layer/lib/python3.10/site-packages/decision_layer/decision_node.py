import os
import shlex
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        package_share_dir = get_package_share_directory('decision_layer')
        workspace_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(package_share_dir)))
        )
        default_spoof_script = os.path.join(workspace_root, 'src', 'Gps_spooferv1.py')
        default_spoof_command = (
            f'python3 {default_spoof_script}' if os.path.exists(default_spoof_script) else ''
        )

        self.declare_parameter('vision_detected_topic', '/vision/detected')
        self.declare_parameter('vision_confidence_topic', '/vision/confidence')
        self.declare_parameter('sound_detected_topic', '/sound/detected')
        self.declare_parameter('sound_confidence_topic', '/sound/confidence')
        self.declare_parameter('warning_topic', '/decision/warning')
        self.declare_parameter('decision_score_topic', '/decision/score')
        self.declare_parameter('spoof_triggered_topic', '/decision/spoof_triggered')
        self.declare_parameter('decision_reason_topic', '/decision/reason')
        self.declare_parameter('decision_state_topic', '/decision/state')
        self.declare_parameter('warning_level_topic', '/decision/warning_level')
        self.declare_parameter('vision_weight', 0.6)
        self.declare_parameter('sound_weight', 0.4)
        self.declare_parameter('min_vision_confidence', 0.2)
        self.declare_parameter('min_sound_confidence', 0.2)
        self.declare_parameter('warning_score_threshold', 0.35)
        self.declare_parameter('spoof_score_threshold', 0.55)
        self.declare_parameter('sensor_timeout_sec', 2.0)
        self.declare_parameter('evaluation_period_sec', 0.2)
        self.declare_parameter('require_both_modalities_for_spoof', True)
        self.declare_parameter('gps_spoof_command', default_spoof_command)
        self.declare_parameter('gps_spoof_cooldown_sec', 10.0)
        self.declare_parameter('max_spoof_triggers', 1)
        self.declare_parameter('reset_trigger_count_when_threat_clears', True)
        self.declare_parameter('reset_cooldown_when_threat_clears', True)

        vision_detected_topic = str(self.get_parameter('vision_detected_topic').value)
        vision_confidence_topic = str(self.get_parameter('vision_confidence_topic').value)
        sound_detected_topic = str(self.get_parameter('sound_detected_topic').value)
        sound_confidence_topic = str(self.get_parameter('sound_confidence_topic').value)
        warning_topic = str(self.get_parameter('warning_topic').value)
        decision_score_topic = str(self.get_parameter('decision_score_topic').value)
        spoof_triggered_topic = str(self.get_parameter('spoof_triggered_topic').value)
        decision_reason_topic = str(self.get_parameter('decision_reason_topic').value)
        decision_state_topic = str(self.get_parameter('decision_state_topic').value)
        warning_level_topic = str(self.get_parameter('warning_level_topic').value)

        self.vision_weight = float(self.get_parameter('vision_weight').value)
        self.sound_weight = float(self.get_parameter('sound_weight').value)
        self.min_vision_confidence = float(self.get_parameter('min_vision_confidence').value)
        self.min_sound_confidence = float(self.get_parameter('min_sound_confidence').value)
        self.warning_score_threshold = float(
            self.get_parameter('warning_score_threshold').value
        )
        self.spoof_score_threshold = float(
            self.get_parameter('spoof_score_threshold').value
        )
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.require_both_modalities_for_spoof = bool(
            self.get_parameter('require_both_modalities_for_spoof').value
        )
        self.gps_spoof_command = str(self.get_parameter('gps_spoof_command').value).strip()
        self.gps_spoof_cooldown_sec = float(
            self.get_parameter('gps_spoof_cooldown_sec').value
        )
        self.max_spoof_triggers = int(self.get_parameter('max_spoof_triggers').value)
        self.reset_trigger_count_when_threat_clears = bool(
            self.get_parameter('reset_trigger_count_when_threat_clears').value
        )
        self.reset_cooldown_when_threat_clears = bool(
            self.get_parameter('reset_cooldown_when_threat_clears').value
        )

        self.vision_detected = False
        self.vision_confidence = 0.0
        self.sound_detected = False
        self.sound_confidence = 0.0
        self.vision_last_time = 0.0
        self.sound_last_time = 0.0
        self.last_state = ''
        self.last_warning_level = ''
        self.last_reason = ''
        self.last_spoof_time = 0.0
        self.spoof_trigger_count = 0
        self.spoof_process = None
        self.last_spoof_block_reason = ''
        self.declare_parameter('stop_spoofing_when_threat_clears', True)
        self.stop_spoofing_when_threat_clears = bool(
            self.get_parameter('stop_spoofing_when_threat_clears').value
        )
        self.threat_present_last_cycle = False

        self.warning_pub = self.create_publisher(Bool, warning_topic, 10)
        self.score_pub = self.create_publisher(Float32, decision_score_topic, 10)
        self.spoof_pub = self.create_publisher(Bool, spoof_triggered_topic, 10)
        self.reason_pub = self.create_publisher(String, decision_reason_topic, 10)
        self.state_pub = self.create_publisher(String, decision_state_topic, 10)
        self.warning_level_pub = self.create_publisher(String, warning_level_topic, 10)

        self.create_subscription(Bool, vision_detected_topic, self.vision_detected_callback, 10)
        self.create_subscription(
            Float32, vision_confidence_topic, self.vision_confidence_callback, 10
        )
        self.create_subscription(Bool, sound_detected_topic, self.sound_detected_callback, 10)
        self.create_subscription(
            Float32, sound_confidence_topic, self.sound_confidence_callback, 10
        )

        evaluation_period_sec = max(
            0.05, float(self.get_parameter('evaluation_period_sec').value)
        )
        self.timer = self.create_timer(evaluation_period_sec, self.evaluate_decision)

        self.get_logger().info(
            'DecisionNode started. '
            f'vision_topics=({vision_detected_topic}, {vision_confidence_topic}), '
            f'sound_topics=({sound_detected_topic}, {sound_confidence_topic}), '
            f'require_both_modalities_for_spoof={self.require_both_modalities_for_spoof}, '
            f'gps_spoof_command={"<configured>" if self.gps_spoof_command else "<empty>"}'
        )

    def now_sec(self) -> float:
        return time.monotonic()

    def vision_detected_callback(self, msg: Bool) -> None:
        self.vision_detected = bool(msg.data)
        self.vision_last_time = self.now_sec()

    def vision_confidence_callback(self, msg: Float32) -> None:
        self.vision_confidence = float(msg.data)
        self.vision_last_time = self.now_sec()

    def sound_detected_callback(self, msg: Bool) -> None:
        self.sound_detected = bool(msg.data)
        self.sound_last_time = self.now_sec()

    def sound_confidence_callback(self, msg: Float32) -> None:
        self.sound_confidence = float(msg.data)
        self.sound_last_time = self.now_sec()

    def is_fresh(self, last_time: float) -> bool:
        return last_time > 0.0 and (self.now_sec() - last_time) <= self.sensor_timeout_sec

    def compute_modal_score(
        self,
        fresh: bool,
        detected: bool,
        confidence: float,
        min_confidence: float,
    ) -> float:
        if not fresh or not detected:
            return 0.0
        if confidence < min_confidence:
            return 0.0
        return max(0.0, min(confidence, 1.0))

    def in_cooldown(self) -> bool:
        return (
            self.last_spoof_time > 0.0 and
            (self.now_sec() - self.last_spoof_time) < self.gps_spoof_cooldown_sec
        )

    def can_trigger_spoofing(self) -> tuple[bool, str]:
        if not self.gps_spoof_command:
            return False, 'gps_command_missing'
        if self.max_spoof_triggers >= 0 and self.spoof_trigger_count >= self.max_spoof_triggers:
            return False, 'max_triggers_reached'
        if self.in_cooldown():
            return False, 'cooldown_active'
        return True, 'ready'

    def trigger_gps_spoofing(self) -> bool:
        can_trigger, block_reason = self.can_trigger_spoofing()
        self.last_spoof_block_reason = block_reason
        if not can_trigger:
            return False

        try:
            command = shlex.split(self.gps_spoof_command)
            self.spoof_process = subprocess.Popen(command)
            self.last_spoof_time = self.now_sec()
            self.spoof_trigger_count += 1
            self.last_spoof_block_reason = 'triggered'
            self.get_logger().warn(
                f'GPS spoofing command triggered: {self.gps_spoof_command}'
            )
            return True
        except Exception as exc:
            self.last_spoof_block_reason = 'trigger_failed'
            self.get_logger().error(f'Failed to start GPS spoofing command: {exc}')
            return False

    def is_spoof_process_running(self) -> bool:
        return self.spoof_process is not None and self.spoof_process.poll() is None

    def stop_gps_spoofing(self, reason: str) -> bool:
        if not self.is_spoof_process_running():
            return False

        try:
            self.spoof_process.terminate()
            self.spoof_process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.spoof_process.kill()
            self.spoof_process.wait(timeout=1.0)
        finally:
            self.spoof_process = None

        self.get_logger().warn(f'GPS spoofing command stopped: {reason}')
        return True

    def clear_spoofing_latch(self) -> None:
        did_reset = False
        if self.reset_trigger_count_when_threat_clears and self.spoof_trigger_count != 0:
            self.spoof_trigger_count = 0
            did_reset = True
        if self.reset_cooldown_when_threat_clears and self.last_spoof_time != 0.0:
            self.last_spoof_time = 0.0
            did_reset = True
        if did_reset:
            self.get_logger().info(
                'Threat cleared. GPS spoofing latch reset and system re-armed.'
            )

    def determine_warning_level(
        self,
        vision_score: float,
        sound_score: float,
        decision_score: float,
        spoof_condition: bool,
    ) -> str:
        any_signal = vision_score > 0.0 or sound_score > 0.0
        if not any_signal:
            return 'none'
        if spoof_condition and decision_score >= self.spoof_score_threshold:
            return 'high'
        if decision_score >= self.warning_score_threshold:
            return 'medium'
        return 'low'

    def determine_state(
        self,
        warning_level: str,
        spoof_condition: bool,
        spoof_triggered: bool,
    ) -> str:
        if spoof_triggered:
            return 'SPOOFING'
        if self.in_cooldown():
            return 'COOLDOWN'
        if warning_level == 'high' or (warning_level == 'medium' and spoof_condition):
            return 'WARNING'
        if warning_level == 'low' or warning_level == 'medium':
            return 'MONITORING'
        return 'IDLE'

    def publish_state(
        self,
        warning: bool,
        decision_score: float,
        spoof_triggered: bool,
        reason: str,
        state: str,
        warning_level: str,
    ) -> None:
        warning_msg = Bool()
        warning_msg.data = bool(warning)
        self.warning_pub.publish(warning_msg)

        score_msg = Float32()
        score_msg.data = float(decision_score)
        self.score_pub.publish(score_msg)

        spoof_msg = Bool()
        spoof_msg.data = bool(spoof_triggered)
        self.spoof_pub.publish(spoof_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        warning_level_msg = String()
        warning_level_msg.data = warning_level
        self.warning_level_pub.publish(warning_level_msg)

    def evaluate_decision(self) -> None:
        vision_fresh = self.is_fresh(self.vision_last_time)
        sound_fresh = self.is_fresh(self.sound_last_time)

        vision_score = self.compute_modal_score(
            vision_fresh,
            self.vision_detected,
            self.vision_confidence,
            self.min_vision_confidence,
        )
        sound_score = self.compute_modal_score(
            sound_fresh,
            self.sound_detected,
            self.sound_confidence,
            self.min_sound_confidence,
        )

        decision_score = (
            self.vision_weight * vision_score +
            self.sound_weight * sound_score
        )

        spoof_condition = (
            vision_score > 0.0 and sound_score > 0.0
            if self.require_both_modalities_for_spoof
            else (vision_score > 0.0 or sound_score > 0.0)
        )
        warning_level = self.determine_warning_level(
            vision_score,
            sound_score,
            decision_score,
            spoof_condition,
        )
        warning = warning_level in ('medium', 'high')
        threat_present = warning_level == 'high' and spoof_condition

        spoof_triggered = False
        if threat_present:
            if self.is_spoof_process_running():
                self.last_spoof_block_reason = 'already_running'
            else:
                spoof_triggered = self.trigger_gps_spoofing()
        else:
            self.last_spoof_block_reason = 'not_requested'
            if self.stop_spoofing_when_threat_clears:
                self.stop_gps_spoofing('threat_cleared')
            if self.threat_present_last_cycle:
                self.clear_spoofing_latch()

        self.threat_present_last_cycle = threat_present

        state = self.determine_state(warning_level, spoof_condition, spoof_triggered)
        reason = (
            f'state={state}, warning_level={warning_level}, '
            f'vision_score={vision_score:.2f}, sound_score={sound_score:.2f}, '
            f'decision_score={decision_score:.2f}, '
            f'vision_fresh={vision_fresh}, sound_fresh={sound_fresh}, '
            f'spoof_status={self.last_spoof_block_reason}'
        )

        self.publish_state(
            warning=warning,
            decision_score=decision_score,
            spoof_triggered=spoof_triggered,
            reason=reason,
            state=state,
            warning_level=warning_level,
        )

        if (
            state != self.last_state or
            warning_level != self.last_warning_level or
            reason != self.last_reason
        ):
            self.get_logger().info(reason)
            self.last_state = state
            self.last_warning_level = warning_level
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_gps_spoofing('node_shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
