import csv
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String

from decision_layer.decision_node import DecisionNode


class FusionStimulus(Node):
    def __init__(self):
        super().__init__("scenario_c_fusion_stimulus")
        self.vision_detected_pub = self.create_publisher(Bool, "/vision/detected", 10)
        self.vision_confidence_pub = self.create_publisher(Float32, "/vision/confidence", 10)
        self.sound_detected_pub = self.create_publisher(Bool, "/sound/detected", 10)
        self.sound_confidence_pub = self.create_publisher(Float32, "/sound/confidence", 10)

    def publish_modalities(
        self,
        vision_detected: bool,
        vision_confidence: float,
        sound_detected: bool,
        sound_confidence: float,
    ):
        vd = Bool()
        vd.data = vision_detected
        vc = Float32()
        vc.data = vision_confidence
        sd = Bool()
        sd.data = sound_detected
        sc = Float32()
        sc.data = sound_confidence

        self.vision_detected_pub.publish(vd)
        self.vision_confidence_pub.publish(vc)
        self.sound_detected_pub.publish(sd)
        self.sound_confidence_pub.publish(sc)


class DecisionMonitor(Node):
    def __init__(self):
        super().__init__("scenario_c_decision_monitor")
        self.warning = None
        self.score = None
        self.spoof_triggered = None
        self.reason = ""
        self.state = ""
        self.warning_level = ""

        self.create_subscription(Bool, "/decision/warning", self.warning_cb, 10)
        self.create_subscription(Float32, "/decision/score", self.score_cb, 10)
        self.create_subscription(Bool, "/decision/spoof_triggered", self.spoof_cb, 10)
        self.create_subscription(String, "/decision/reason", self.reason_cb, 10)
        self.create_subscription(String, "/decision/state", self.state_cb, 10)
        self.create_subscription(String, "/decision/warning_level", self.level_cb, 10)

    def warning_cb(self, msg: Bool):
        self.warning = bool(msg.data)

    def score_cb(self, msg: Float32):
        self.score = float(msg.data)

    def spoof_cb(self, msg: Bool):
        self.spoof_triggered = bool(msg.data)

    def reason_cb(self, msg: String):
        self.reason = msg.data

    def state_cb(self, msg: String):
        self.state = msg.data

    def level_cb(self, msg: String):
        self.warning_level = msg.data

    def snapshot(self):
        return {
            "warning": self.warning,
            "decision_score": self.score,
            "spoof_triggered": self.spoof_triggered,
            "state": self.state,
            "warning_level": self.warning_level,
            "reason": self.reason,
        }


def spin_for(executor: SingleThreadedExecutor, seconds: float):
    end_time = time.monotonic() + seconds
    while time.monotonic() < end_time:
        executor.spin_once(timeout_sec=0.05)


def run_case(executor, stimulus, monitor, case):
    for _ in range(6):
        stimulus.publish_modalities(
            case["vision_detected"],
            case["vision_confidence"],
            case["sound_detected"],
            case["sound_confidence"],
        )
        spin_for(executor, 0.1)

    spin_for(executor, 0.5)
    result = monitor.snapshot()
    return {
        **case,
        **result,
    }


def main():
    rclpy.init()

    decision = DecisionNode()
    decision.set_parameters(
        [
            Parameter("gps_spoof_command", Parameter.Type.STRING, ""),
            Parameter("max_spoof_triggers", Parameter.Type.INTEGER, 0),
        ]
    )
    decision.gps_spoof_command = ""
    decision.max_spoof_triggers = 0

    stimulus = FusionStimulus()
    monitor = DecisionMonitor()

    executor = SingleThreadedExecutor()
    executor.add_node(decision)
    executor.add_node(stimulus)
    executor.add_node(monitor)

    cases = [
        {
            "case": "C1_both_vision_and_sound_positive",
            "vision_detected": True,
            "vision_confidence": 0.80,
            "sound_detected": True,
            "sound_confidence": 1.00,
            "expected": "high warning, spoof condition true",
        },
        {
            "case": "C2_vision_only_positive",
            "vision_detected": True,
            "vision_confidence": 0.80,
            "sound_detected": False,
            "sound_confidence": 0.00,
            "expected": "medium warning, no spoof condition",
        },
        {
            "case": "C3_sound_only_positive",
            "vision_detected": False,
            "vision_confidence": 0.00,
            "sound_detected": True,
            "sound_confidence": 1.00,
            "expected": "medium warning, no spoof condition",
        },
        {
            "case": "C4_no_modality_positive",
            "vision_detected": False,
            "vision_confidence": 0.00,
            "sound_detected": False,
            "sound_confidence": 0.00,
            "expected": "no warning, idle",
        },
    ]

    output_dir = Path("/home/prp/thesis/scenario_c_fusion_eval_results")
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "scenario_c_fusion_eval.csv"

    results = []
    try:
        spin_for(executor, 0.5)
        for case in cases:
            for trial in range(1, 4):
                result = run_case(executor, stimulus, monitor, case)
                result["trial"] = trial
                results.append(result)

        with csv_path.open("w", newline="") as f:
            fieldnames = [
                "case",
                "trial",
                "vision_detected",
                "vision_confidence",
                "sound_detected",
                "sound_confidence",
                "expected",
                "warning",
                "decision_score",
                "spoof_triggered",
                "state",
                "warning_level",
                "reason",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

        print(f"csv={csv_path}")
        for result in results:
            print(
                f'{result["case"]} trial={result["trial"]}: '
                f'score={result["decision_score"]:.2f}, '
                f'level={result["warning_level"]}, warning={result["warning"]}, '
                f'state={result["state"]}, spoof_triggered={result["spoof_triggered"]}'
            )
    finally:
        decision.stop_gps_spoofing("scenario_c_eval_shutdown")
        executor.remove_node(monitor)
        executor.remove_node(stimulus)
        executor.remove_node(decision)
        monitor.destroy_node()
        stimulus.destroy_node()
        decision.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
