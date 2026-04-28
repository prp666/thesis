import csv
import time
from pathlib import Path

import rclpy
from gz.msgs10.navsat_pb2 import NavSat
from gz.transport13 import Node as GzNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from decision_layer.decision_node import DecisionNode


GPS_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat"
GPS_SPOOF_COMMAND = "python3 /home/prp/thesis/thesis_ws/src/Gps_spooferv1.py"


class ThreatStimulus(Node):
    def __init__(self):
        super().__init__("scenario_e_threat_stimulus")
        self.vision_detected_pub = self.create_publisher(Bool, "/vision/detected", 10)
        self.vision_confidence_pub = self.create_publisher(Float32, "/vision/confidence", 10)
        self.sound_detected_pub = self.create_publisher(Bool, "/sound/detected", 10)
        self.sound_confidence_pub = self.create_publisher(Float32, "/sound/confidence", 10)

    def publish(self, vision_detected, vision_confidence, sound_detected, sound_confidence):
        vd = Bool()
        vd.data = bool(vision_detected)
        vc = Float32()
        vc.data = float(vision_confidence)
        sd = Bool()
        sd.data = bool(sound_detected)
        sc = Float32()
        sc.data = float(sound_confidence)
        self.vision_detected_pub.publish(vd)
        self.vision_confidence_pub.publish(vc)
        self.sound_detected_pub.publish(sd)
        self.sound_confidence_pub.publish(sc)


class DecisionMonitor(Node):
    def __init__(self):
        super().__init__("scenario_e_decision_monitor")
        self.warning = None
        self.score = None
        self.spoof_triggered = None
        self.reason = ""
        self.state = ""
        self.warning_level = ""
        self.observed_spoof_triggered = False
        self.observed_spoofing_state = False
        self.observed_cooldown_state = False
        self.create_subscription(Bool, "/decision/warning", self.warning_cb, 10)
        self.create_subscription(Float32, "/decision/score", self.score_cb, 10)
        self.create_subscription(Bool, "/decision/spoof_triggered", self.spoof_cb, 10)
        self.create_subscription(String, "/decision/reason", self.reason_cb, 10)
        self.create_subscription(String, "/decision/state", self.state_cb, 10)
        self.create_subscription(String, "/decision/warning_level", self.level_cb, 10)

    def warning_cb(self, msg):
        self.warning = bool(msg.data)

    def score_cb(self, msg):
        self.score = float(msg.data)

    def spoof_cb(self, msg):
        self.spoof_triggered = bool(msg.data)
        if msg.data:
            self.observed_spoof_triggered = True

    def reason_cb(self, msg):
        self.reason = msg.data

    def state_cb(self, msg):
        self.state = msg.data
        if msg.data == "SPOOFING":
            self.observed_spoofing_state = True
        if msg.data == "COOLDOWN":
            self.observed_cooldown_state = True

    def level_cb(self, msg):
        self.warning_level = msg.data

    def clear_observed_events(self):
        self.observed_spoof_triggered = False
        self.observed_spoofing_state = False
        self.observed_cooldown_state = False


class GzNavSatMonitor:
    def __init__(self, topic):
        self.node = GzNode()
        self.topic = topic
        self.messages = []
        self.node.subscribe(NavSat, topic, self.callback)

    def callback(self, msg):
        self.messages.append(
            {
                "time": time.monotonic(),
                "lat": float(msg.latitude_deg),
                "lon": float(msg.longitude_deg),
                "alt": float(msg.altitude),
            }
        )

    def count(self):
        return len(self.messages)

    def rate_since(self, start_count):
        subset = self.messages[start_count:]
        if len(subset) < 2:
            return 0.0
        duration = subset[-1]["time"] - subset[0]["time"]
        if duration <= 0.0:
            return 0.0
        return (len(subset) - 1) / duration

    def last_position(self):
        if not self.messages:
            return None
        msg = self.messages[-1]
        return f'{msg["lat"]:.8f},{msg["lon"]:.8f},{msg["alt"]:.3f}'


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


def publish_threat_for(executor, stimulus, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        stimulus.publish(True, 0.8, True, 1.0)
        spin_for(executor, 0.1)


def publish_clear_for(executor, stimulus, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        stimulus.publish(False, 0.0, False, 0.0)
        spin_for(executor, 0.1)


def configure_decision_for_eval(decision):
    decision.gps_spoof_command = GPS_SPOOF_COMMAND
    decision.gps_spoof_cooldown_sec = 1.0
    decision.max_spoof_triggers = -1
    decision.sensor_timeout_sec = 0.6
    decision.require_both_modalities_for_spoof = True
    decision.warning_score_threshold = 0.35
    decision.spoof_score_threshold = 0.55
    decision.min_vision_confidence = 0.2
    decision.min_sound_confidence = 0.2
    decision.reset_trigger_count_when_threat_clears = True
    decision.reset_cooldown_when_threat_clears = True
    decision.stop_spoofing_when_threat_clears = True


def make_row(case, purpose, expected, decision, monitor, gz_monitor, start_count, end_count):
    new_messages = end_count - start_count
    return {
        "case": case,
        "purpose": purpose,
        "expected": expected,
        "decision_state": monitor.state,
        "warning_level": monitor.warning_level,
        "decision_score": monitor.score,
        "observed_spoof_triggered": monitor.observed_spoof_triggered,
        "observed_spoofing_state": monitor.observed_spoofing_state,
        "observed_cooldown_state": monitor.observed_cooldown_state,
        "process_running": decision.is_spoof_process_running(),
        "gz_messages_observed": new_messages,
        "gz_publish_rate_hz": gz_monitor.rate_since(start_count),
        "last_spoofed_position": gz_monitor.last_position() or "",
        "reason": monitor.reason,
    }


def main():
    rclpy.init()
    decision = DecisionNode()
    configure_decision_for_eval(decision)
    stimulus = ThreatStimulus()
    monitor = DecisionMonitor()
    gz_monitor = GzNavSatMonitor(GPS_TOPIC)

    executor = SingleThreadedExecutor()
    executor.add_node(decision)
    executor.add_node(stimulus)
    executor.add_node(monitor)

    rows = []
    try:
        spin_for(executor, 0.5)

        monitor.clear_observed_events()
        start = gz_monitor.count()
        publish_threat_for(executor, stimulus, 1.0)
        spin_for(executor, 0.3)
        end = gz_monitor.count()
        rows.append(
            make_row(
                "E1_action_activation",
                "Verify that high-threat input starts the GPS spoofing process and injection path.",
                "trigger observed, process running, Gazebo NavSat messages observed",
                decision,
                monitor,
                gz_monitor,
                start,
                end,
            )
        )

        monitor.clear_observed_events()
        start = gz_monitor.count()
        publish_threat_for(executor, stimulus, 2.0)
        end = gz_monitor.count()
        rows.append(
            make_row(
                "E2_action_persistence",
                "Verify that spoofing continues publishing instead of only triggering once.",
                "continuous Gazebo NavSat messages near the configured 50 Hz rate",
                decision,
                monitor,
                gz_monitor,
                start,
                end,
            )
        )

        monitor.clear_observed_events()
        start = gz_monitor.count()
        publish_clear_for(executor, stimulus, 1.0)
        spin_for(executor, 0.7)
        mid = gz_monitor.count()
        process_after_clear = decision.is_spoof_process_running()
        spin_for(executor, 0.7)
        end = gz_monitor.count()
        row = make_row(
            "E3_action_deactivation",
            "Verify that removing the threat stops the spoofing process and injection stream.",
            "process stopped, state falls back, no additional Gazebo messages after stop",
            decision,
            monitor,
            gz_monitor,
            mid,
            end,
        )
        row["process_running_immediately_after_clear"] = process_after_clear
        row["messages_during_clear_window"] = mid - start
        row["messages_after_stop_window"] = end - mid
        rows.append(row)

        monitor.clear_observed_events()
        start = gz_monitor.count()
        publish_threat_for(executor, stimulus, 1.2)
        spin_for(executor, 0.4)
        end = gz_monitor.count()
        rows.append(
            make_row(
                "E4_rearming_reversibility",
                "Verify that the action layer can be re-armed and activated again after deactivation.",
                "second trigger observed and new Gazebo NavSat messages published",
                decision,
                monitor,
                gz_monitor,
                start,
                end,
            )
        )

        output_dir = Path("/home/prp/thesis/scenario_e_action_eval_results")
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / "scenario_e_action_layer_eval.csv"
        fieldnames = [
            "case",
            "purpose",
            "expected",
            "decision_state",
            "warning_level",
            "decision_score",
            "observed_spoof_triggered",
            "observed_spoofing_state",
            "observed_cooldown_state",
            "process_running",
            "process_running_immediately_after_clear",
            "gz_messages_observed",
            "messages_during_clear_window",
            "messages_after_stop_window",
            "gz_publish_rate_hz",
            "last_spoofed_position",
            "reason",
        ]
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

        print(f"csv={csv_path}")
        for row in rows:
            print(
                f'{row["case"]}: trigger={row["observed_spoof_triggered"]}, '
                f'process_running={row["process_running"]}, '
                f'gz_msgs={row["gz_messages_observed"]}, '
                f'rate={row["gz_publish_rate_hz"]:.1f} Hz, '
                f'state={row["decision_state"]}'
            )
    finally:
        decision.stop_gps_spoofing("scenario_e_eval_shutdown")
        executor.remove_node(monitor)
        executor.remove_node(stimulus)
        executor.remove_node(decision)
        monitor.destroy_node()
        stimulus.destroy_node()
        decision.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
