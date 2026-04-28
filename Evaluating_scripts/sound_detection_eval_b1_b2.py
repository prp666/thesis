import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Int16MultiArray

from sound_detection.fake_audio_source_node import FakeAudioSourceNode
from sound_detection.sound_presence_detector_node import SoundPresenceDetectorNode


class SoundEvalMonitor(Node):
    def __init__(self, sample_rate: int, energy_threshold: float, ratio_threshold: float):
        super().__init__("sound_eval_monitor")
        self.sample_rate = sample_rate
        self.energy_threshold = energy_threshold
        self.ratio_threshold = ratio_threshold
        self.latest_detected = False
        self.latest_confidence = 0.0
        self.records = []

        self.create_subscription(Int16MultiArray, "/sound/audio", self.audio_callback, 10)
        self.create_subscription(Bool, "/sound/detected", self.detected_callback, 10)
        self.create_subscription(Float32, "/sound/confidence", self.confidence_callback, 10)

    def detected_callback(self, msg: Bool):
        self.latest_detected = bool(msg.data)

    def confidence_callback(self, msg: Float32):
        self.latest_confidence = float(msg.data)

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

        self.records.append(
            {
                "t_sec": time.monotonic(),
                "energy": energy,
                "band_ratio": band_ratio,
                "raw_detected": raw_detected,
                "detected": self.latest_detected,
                "confidence": self.latest_confidence,
            }
        )


def summarize(records):
    if not records:
        return {}

    stable = records[5:] if len(records) > 5 else records
    energies = np.array([r["energy"] for r in stable], dtype=np.float64)
    ratios = np.array([r["band_ratio"] for r in stable], dtype=np.float64)
    confidences = np.array([r["confidence"] for r in stable], dtype=np.float64)
    raw = np.array([r["raw_detected"] for r in stable], dtype=np.bool_)
    detected = np.array([r["detected"] for r in stable], dtype=np.bool_)

    return {
        "frames": len(records),
        "stable_frames": len(stable),
        "energy_mean": float(np.mean(energies)),
        "energy_min": float(np.min(energies)),
        "energy_max": float(np.max(energies)),
        "band_ratio_mean": float(np.mean(ratios)),
        "band_ratio_min": float(np.min(ratios)),
        "band_ratio_max": float(np.max(ratios)),
        "confidence_mean": float(np.mean(confidences)),
        "confidence_min": float(np.min(confidences)),
        "confidence_max": float(np.max(confidences)),
        "raw_detected_rate": float(np.mean(raw)),
        "detected_rate": float(np.mean(detected)),
        "final_detected": bool(records[-1]["detected"]),
        "final_confidence": float(records[-1]["confidence"]),
    }


def run_case(case_name: str, drone_present: bool, duration_sec: float, output_dir: Path):
    source = FakeAudioSourceNode()
    detector = SoundPresenceDetectorNode()
    source.set_parameters([Parameter("drone_present", Parameter.Type.BOOL, drone_present)])

    monitor = SoundEvalMonitor(
        sample_rate=detector.sample_rate,
        energy_threshold=detector.energy_threshold,
        ratio_threshold=detector.ratio_threshold,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(detector)
    executor.add_node(monitor)

    end_time = time.monotonic() + duration_sec
    while time.monotonic() < end_time:
        executor.spin_once(timeout_sec=0.05)

    executor.remove_node(monitor)
    executor.remove_node(detector)
    executor.remove_node(source)
    monitor.destroy_node()
    detector.destroy_node()
    source.destroy_node()

    csv_path = output_dir / f"{case_name}.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t_sec",
                "energy",
                "band_ratio",
                "raw_detected",
                "detected",
                "confidence",
            ],
        )
        writer.writeheader()
        writer.writerows(monitor.records)

    return summarize(monitor.records), csv_path


def main():
    rclpy.init()
    output_dir = Path("/home/prp/thesis/sound_detection_eval_results")
    output_dir.mkdir(parents=True, exist_ok=True)

    cases = [
        ("B1_uav_like_sound_present", True),
        ("B2_background_noise_only", False),
    ]

    print("case,frames,energy_mean,energy_min,energy_max,band_ratio_mean,band_ratio_min,band_ratio_max,confidence_mean,confidence_min,confidence_max,raw_detected_rate,detected_rate,final_detected,final_confidence,csv")
    try:
        for case_name, drone_present in cases:
            summary, csv_path = run_case(case_name, drone_present, 5.0, output_dir)
            print(
                ",".join(
                    [
                        case_name,
                        str(summary.get("frames", 0)),
                        f'{summary.get("energy_mean", math.nan):.2f}',
                        f'{summary.get("energy_min", math.nan):.2f}',
                        f'{summary.get("energy_max", math.nan):.2f}',
                        f'{summary.get("band_ratio_mean", math.nan):.4f}',
                        f'{summary.get("band_ratio_min", math.nan):.4f}',
                        f'{summary.get("band_ratio_max", math.nan):.4f}',
                        f'{summary.get("confidence_mean", math.nan):.3f}',
                        f'{summary.get("confidence_min", math.nan):.3f}',
                        f'{summary.get("confidence_max", math.nan):.3f}',
                        f'{summary.get("raw_detected_rate", math.nan):.3f}',
                        f'{summary.get("detected_rate", math.nan):.3f}',
                        str(summary.get("final_detected", False)),
                        f'{summary.get("final_confidence", math.nan):.3f}',
                        str(csv_path),
                    ]
                )
            )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
