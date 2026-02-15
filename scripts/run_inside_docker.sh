#!/usr/bin/env bash
set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"
ws_root="${WS_ROOT:-/workspace}"

required_paths=(
  "${ws_root}/storm"
  "${ws_root}/IsaacLab"
  "${ws_root}/ros2_ws/src/storm_agimus_bridge"
  "${ws_root}/ros2_ws/src/storm_pick_place_demo"
)

for path in "${required_paths[@]}"; do
  if [[ ! -e "${path}" ]]; then
    echo "Missing required path: ${path}"
    echo "Run scripts/populate_workspace.sh on host before docker build."
    exit 1
  fi
done

python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install --extra-index-url https://pypi.nvidia.com isaacsim==4.5.0.0
python3 -m pip install --upgrade --ignore-installed "sympy>=1.13.0" "mpmath>=1.3.0"
python3 -m pip install -e "${ws_root}/IsaacLab/source/isaaclab" \
  -e "${ws_root}/IsaacLab/source/isaaclab_assets" \
  -e "${ws_root}/IsaacLab/source/isaaclab_mimic" \
  -e "${ws_root}/IsaacLab/source/isaaclab_rl" \
  -e "${ws_root}/IsaacLab/source/isaaclab_tasks"
python3 -m pip install -e "${ws_root}/storm"

rosdep init 2>/dev/null || true
rosdep update

# ROS setup scripts are not always nounset-safe.
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u
apt-get update
apt-get install -y \
  ros-humble-gtest-vendor ros-humble-gmock-vendor \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-gmock \
  libgtest-dev libgmock-dev
rosdep install --from-paths "${ws_root}/ros2_ws/src" --ignore-src -r -y --rosdistro "${ros_distro}" \
  -t build -t buildtool -t exec -t test \
  --skip-keys "example-robot-data net_ft_driver net_ft_description net_ft_diagnostic_broadcaster"

cd "${ws_root}/ros2_ws"
rm -rf build install log
colcon build --symlink-install
