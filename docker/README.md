# Docker ROS2 Pipeline

This Docker setup runs the thesis ROS2 pipeline without Gazebo or QGroundControl:

- fake image source
- vision detector
- fake audio source
- sound detector
- decision node
- action stub

The folder is self-contained for Docker builds: it includes its own `thesis_ws/src`
copy with the ROS2 packages needed by this container.

Build and run:

```bash
cd docker
docker compose up --build thesis_pipeline
```

Open a shell in the same image:

```bash
cd docker
docker compose --profile tools run --rm shell
```

Useful topic checks from another terminal:

```bash
cd docker
docker compose --profile tools run --rm shell ros2 topic list
docker compose --profile tools run --rm shell ros2 topic echo /decision/reason
docker compose --profile tools run --rm shell ros2 topic echo /action/status
```

The Docker decision config uses `/bin/sleep 3600` as the command trigger. That keeps the decision/action path testable without starting the Gazebo GPS spoofing script.
