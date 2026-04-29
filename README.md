# Thesis ROS2 Threat Detection Pipeline

This repository contains a ROS2 Humble thesis prototype for a UAV/drone threat
detection and response pipeline. The system combines sound detection and vision
detection, fuses both signals in a decision layer, and can trigger an action
layer response.

The repository also includes a Docker version that runs the core pipeline
without Gazebo or QGroundControl. This is the recommended way to reproduce the
main software pipeline on another machine.

## Repository Structure

```text
.
├── thesis_ws/
│   └── src/
│       ├── sound_detection/
│       ├── yolo_detection/
│       ├── decision_layer/
│       └── Gps_spooferv1.py
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── README.md
│   └── thesis_ws/src/
├── Evaluating_scripts/
└── Results/
```

## Main Components

`sound_detection`

ROS2 Python package for the audio side of the pipeline.

- `fake_audio_source_node`: publishes synthetic UAV-like audio on `/sound/audio`
- `sound_presence_detector_node`: detects UAV-like sound and publishes:
  - `/sound/detected`
  - `/sound/confidence`

`yolo_detection`

ROS2 Python package for the vision side of the pipeline.

- `fake_image_source_node`: publishes image frames on `/camera/image_raw`
- `vision_detector`: runs a TFLite YOLO detector and publishes:
  - `/vision/detected`
  - `/vision/confidence`
  - `/vision/bbox`
  - `/vision/class_id`

`decision_layer`

ROS2 Python package that fuses sound and vision detections.

- subscribes to `/sound/*` and `/vision/*`
- publishes:
  - `/decision/warning`
  - `/decision/score`
  - `/decision/spoof_triggered`
  - `/decision/reason`
  - `/decision/state`
  - `/decision/warning_level`

`Gps_spooferv1.py`

Gazebo GPS spoofing script used by the full action-layer experiment. This is not
used by the Docker-only pipeline.

`docker/`

Self-contained Docker version of the core ROS2 pipeline. It includes its own
copy of the required ROS2 packages under `docker/thesis_ws/src`.

The Docker pipeline runs:

- fake image source
- vision detector
- fake audio source
- sound detector
- decision node
- action stub

It intentionally excludes Gazebo and QGroundControl.

## Quick Start With Docker

Install Docker and Docker Compose, then run:

```bash
cd docker
docker compose up --build thesis_pipeline
```

Open another terminal to inspect ROS2 topics:

```bash
cd docker
docker compose --profile tools run --rm shell ros2 topic list
docker compose --profile tools run --rm shell ros2 topic echo /decision/reason
docker compose --profile tools run --rm shell ros2 topic echo /action/status
```

More Docker details are in [docker/README.md](docker/README.md).

## Local ROS2 Usage

If you want to run the original workspace directly, use ROS2 Humble:

```bash
cd thesis_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Run the demo launch file:

```bash
ros2 launch decision_layer threat_response_demo.launch.py
```

Run only the decision node:

```bash
ros2 launch decision_layer decision_node.launch.py
```

Run vision components separately:

```bash
ros2 launch yolo_detection fake_image_source.launch.py
ros2 launch yolo_detection vision_detector.launch.py
```

## Evaluation Scripts

The evaluation scripts are stored in `Evaluating_scripts/`.

Examples:

```bash
python3 Evaluating_scripts/sound_detection_eval_b1_b2.py
python3 Evaluating_scripts/scenario_c_fusion_eval.py
python3 Evaluating_scripts/scenario_d_decision_logic_eval.py
python3 Evaluating_scripts/scenario_e_action_layer_eval.py
bash Evaluating_scripts/scenario_f_runtime_observer.sh
```

Generated/recorded outputs are stored in `Results/`.

Important result folders:

- `Results/sound_detection_eval_results/`
- `Results/scenario_c_fusion_eval_results/`
- `Results/scenario_d_decision_eval_results/`
- `Results/scenario_e_action_eval_results/`
- `Results/scenario_f_runtime_eval_results/`

## Expected Pipeline Behavior

The core data flow is:

```text
fake_audio_source_node -> sound_presence_detector_node -> decision_node
fake_image_source_node -> vision_detector              -> decision_node
decision_node          -> action layer
```

For high-confidence sound and vision detections, the decision node should publish
a high warning level and a decision score above the spoof/action threshold. In
the Docker version, the action layer is a stub that confirms the decision trigger
without starting Gazebo-based GPS spoofing.

## Notes

- The Docker setup is the easiest way to reproduce the software-only pipeline.
- The original `thesis_ws/` workspace contains the thesis ROS2 source packages.
- Gazebo/QGroundControl integration is outside the Docker pipeline and belongs
  to the full local simulation/action experiment.
