# Cosmik MPPI Docker Setup

Docker container for Cosmik MPPI with Isaac Sim, Isaac Lab, STORM, and ROS 2 Humble.

## Prerequisites

- Ubuntu 22.04 host
- NVIDIA RTX-generation GPU (Isaac Sim prerequisite)
- Docker + Docker Compose plugin (`docker compose`)
- NVIDIA driver + NVIDIA Container Toolkit (for `--gpus all`)
- X11 available on host (`/tmp/.X11-unix`, `DISPLAY`)

## Installation / Build

From repository root:

```bash
chmod +x scripts/*.sh
bash scripts/build_all.sh
```

What this does:
1. Populates `./workspace` with pinned source repositories (`populate_workspace.sh`).
2. Builds the Docker image `cosmik-mppi:latest` using `docker/Dockerfile`.
3. Starts container `cosmik-mppi` through `docker/docker-compose.yml` with `./workspace` bind-mounted to `/workspace` inside the container.
4. The container is not removed on exit by default.

## Manual Commands (Optional)

If you want to run steps separately:

```bash
# 1) Populate workspace
bash scripts/populate_workspace.sh ./workspace

# 2) Build image
docker build -t cosmik-mppi -f docker/Dockerfile .

# 3) Run container
docker compose -f docker/docker-compose.yml run cosmik-mppi
```

## Notes

- To start a new interactive session with Compose:
  - `docker compose -f docker/docker-compose.yml run cosmik-mppi`
- To reconnect to the existing named container:
  - `docker start cosmik-mppi` (if it is stopped)
  - `docker exec -it cosmik-mppi bash`
- You can also use VS Code: **Dev Containers: Attach to Running Container...** and select `cosmik-mppi`.
- `run_inside_docker.sh` is executed during image build to install dependencies and run `colcon build`.
- Runtime environment variables are set in the image:
  - `STORM_ROOT=/workspace/storm`
  - `PYTHONPATH=/workspace/storm`

## After Entering the Container

Before running ROS 2 commands:

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

## Run Pick-and-Place Demo (Simulation)

```bash
ros2 launch storm_pick_place_demo pick_place_sim_isaacsim.launch.py \
  armed:=true with_rviz:=true gz_headless:=true
```

## Run Pick-and-Place Demo (Real Robot)

Safety gate is controlled by `enable_motion` (default is `false`).

```bash
# Dry run (no robot motion)
ros2 launch storm_pick_place_demo pick_place_real.launch.py \
  enable_motion:=false with_rviz:=true

# Enable motion
ros2 launch storm_pick_place_demo pick_place_real.launch.py \
  enable_motion:=true with_rviz:=true
```

## Parameter Files

- Planner bridge + MPPI runtime settings:
  - `/workspace/ros2_ws/src/storm_agimus_bridge/config/storm_mppi_planner_params.yaml`
- STORM MPPI task file selected by planner (`storm_task_file`):
  - `/workspace/storm/content/configs/mpc/franka_reacher.yml`
- Pick/place phase logic, tolerances, gripper open/close targets:
  - `/workspace/ros2_ws/src/storm_pick_place_demo/config/pick_place_task_manager_params.yaml`
- Pick/place objects poses/sizes, marker topic, barrier object:
  - `/workspace/ros2_ws/src/storm_pick_place_demo/config/object_manager_params.yaml`
- Linear feedback controller gains and controller-side parameters:
  - `/workspace/ros2_ws/src/storm_pick_place_demo/config/linear_feedback_controller_params.yaml`
- Demo launch-level controller and interface parameters:
  - `/workspace/ros2_ws/src/storm_pick_place_demo/config/agimus_controller_params.yaml`
