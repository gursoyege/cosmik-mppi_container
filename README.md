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

## Open / Reconnect Container Shell

From repository root:

```bash
bash scripts/run.sh
```

`scripts/run.sh` behavior:
1. If a matching container is already running, it opens `bash` in it.
2. If a matching container exists but is stopped, it starts it and opens `bash`.
3. If no matching container exists but image `cosmik-mppi` exists, it runs `docker compose -f docker/docker-compose.yml up -d cosmik-mppi` and then opens `bash`.
4. If multiple containers exist from image `cosmik-mppi`, it warns and uses the newest one.

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

- Preferred way to enter/re-enter container shell:
  - `bash scripts/run.sh`
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
# Dry run (no motion)
ros2 launch storm_pick_place_demo pick_place_real.launch.py \
  robot_ip:=<ROBOT_IP> arm_id:=fer enable_motion:=false with_rviz:=true

# Enable motion
ros2 launch storm_pick_place_demo pick_place_real.launch.py \
  robot_ip:=<ROBOT_IP> arm_id:=fer enable_motion:=true with_rviz:=true
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
