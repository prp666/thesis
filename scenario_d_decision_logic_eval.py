import csv
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from decision_layer.decision_node import DecisionNode


class DecisionStimulus(Node):
    def __init__(self):
        super().__init__("scenario_d_decision_stimulus")
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
        super().__init__("scenario_d_decision_monitor")
        self.warning = None
        self.score = None
        self.spoof_triggered = None
        self.reason = ""
        self.state = ""
        self.warning_level = ""
        self.observed_spoof_triggered = False
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

    def level_cb(self, msg):
        self.warning_level = msg.data

    def snapshot(self, case, expected):
        return {
            "case": case,
            "expected": expected,
            "warning": self.warning,
            "decision_score": self.score,
            "spoof_triggered": self.spoof_triggered,
            "observed_spoof_triggered": self.observed_spoof_triggered,
            "state": self.state,
            "warning_level": self.warning_level,
            "reason": self.reason,
        }

    def clear_observed_events(self):
        self.observed_spoof_triggered = False


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


def publish_for(executor, stimulus, seconds, values):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        stimulus.publish(*values)
        spin_for(executor, 0.1)


def configure_decision_for_eval(decision):
    decision.gps_spoof_command = ""
    decision.gps_spoof_cooldown_sec = 2.0
    decision.max_spoof_triggers = -1
    decision.sensor_timeout_sec = 0.6
    decision.evaluation_period_sec = 0.1
    decision.require_both_modalities_for_spoof = True
    decision.warning_score_threshold = 0.35
    decision.spoof_score_threshold = 0.55
    decision.min_vision_confidence = 0.2
    decision.min_sound_confidence = 0.2
    decision.reset_trigger_count_when_threat_clears = True
    decision.reset_cooldown_when_threat_clears = True
    decision.stop_spoofing_when_threat_clears = True


def main():
    rclpy.init()
    decision = DecisionNode()
    configure_decision_for_eval(decision)
    stimulus = DecisionStimulus()
    monitor = DecisionMonitor()

    executor = SingleThreadedExecutor()
    executor.add_node(decision)
    executor.add_node(stimulus)
    executor.add_node(monitor)

    results = []
    try:
        spin_for(executor, 0.4)

        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (True, 0.8, True, 1.0))
        results.append(
            monitor.snapshot(
                "D1a_fresh_both_positive",
                "fresh data should produce high warning and valid threat without invoking the external trigger in this subtest",
            )
        )

        monitor.clear_observed_events()
        spin_for(executor, 1.0)
        results.append(
            monitor.snapshot(
                "D1b_after_freshness_timeout",
                "without new sensor messages, both modalities should become stale and state should fall back to IDLE",
            )
        )

        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (True, 0.19, True, 1.0))
        results.append(
            monitor.snapshot(
                "D2a_below_min_vision_confidence",
                "vision should be gated out; sound alone gives medium monitoring only",
            )
        )

        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (True, 0.25, True, 1.0))
        results.append(
            monitor.snapshot(
                "D2b_at_spoof_score_threshold",
                "combined score reaches 0.55 and both modalities satisfy the high-warning gate",
            )
        )

        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (False, 0.0, True, 1.0))
        results.append(
            monitor.snapshot(
                "D3_gating_sound_only",
                "single modality should not satisfy require_both_modalities_for_spoof",
            )
        )

        decision.gps_spoof_command = "/usr/bin/true"
        decision.last_spoof_time = 0.0
        decision.spoof_trigger_count = 0
        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (True, 0.8, True, 1.0))
        results.append(
            monitor.snapshot(
                "D4a_first_trigger",
                "first high-confidence threat should trigger spoof command",
            )
        )

        monitor.clear_observed_events()
        publish_for(executor, stimulus, 0.8, (True, 0.8, True, 1.0))
        results.append(
            monitor.snapshot(
                "D4b_cooldown_active",
                "continued threat during cooldown should not trigger again and should report cooldown",
            )
        )

        monitor.clear_observed_events()
        spin_for(executor, 1.0)
        results.append(
            monitor.snapshot(
                "D4c_timeout_state_fallback",
                "when threat messages stop, state should return to IDLE",
            )
        )

        output_dir = Path("/home/prp/thesis/scenario_d_decision_eval_results")
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / "scenario_d_decision_logic_eval.csv"
        with csv_path.open("w", newline="") as f:
            fieldnames = [
                "case",
                "expected",
                "warning",
                "decision_score",
                "spoof_triggered",
                "observed_spoof_triggered",
                "state",
                "warning_level",
                "reason",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

        print(f"csv={csv_path}")
        for result in results:
            score = result["decision_score"]
            score_text = "None" if score is None else f"{score:.2f}"
            print(
                f'{result["case"]}: score={score_text}, '
                f'level={result["warning_level"]}, warning={result["warning"]}, '
                f'state={result["state"]}, spoof_triggered={result["spoof_triggered"]}, '
                f'observed_spoof_triggered={result["observed_spoof_triggered"]}'
            )
    finally:
        decision.stop_gps_spoofing("scenario_d_eval_shutdown")
        executor.remove_node(monitor)
        executor.remove_node(stimulus)
        executor.remove_node(decision)
        monitor.destroy_node()
        stimulus.destroy_node()
        decision.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
