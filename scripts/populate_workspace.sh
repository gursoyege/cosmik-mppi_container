#!/usr/bin/env bash
set -euo pipefail

target_root="${1:-$(pwd)/workspace}"

clone_checkout() {
  local repo_url="$1"
  local commit_sha="$2"
  local dst_dir="$3"

  if [[ -d "${dst_dir}/.git" ]]; then
    git -C "${dst_dir}" fetch --prune --tags origin
  else
    rm -rf "${dst_dir}"
    git clone --no-checkout "${repo_url}" "${dst_dir}"
  fi

  git -C "${dst_dir}" checkout --force "${commit_sha}"
  git -C "${dst_dir}" submodule update --init --recursive
}

keep_only() {
  local parent="$1"
  shift
  local keep=("$@")
  local entry=""
  local base=""
  local should_keep=0

  shopt -s dotglob nullglob
  for entry in "${parent}"/*; do
    base="$(basename "${entry}")"
    should_keep=0
    if [[ "${base}" == ".git" ]]; then
      should_keep=1
    else
      for item in "${keep[@]}"; do
        if [[ "${base}" == "${item}" ]]; then
          should_keep=1
          break
        fi
      done
    fi
    if [[ "${should_keep}" -eq 0 ]]; then
      rm -rf "${entry}"
    fi
  done
  shopt -u dotglob nullglob
}

mkdir -p "${target_root}" "${target_root}/ros2_ws/src"

clone_checkout "https://github.com/gursoyege/storm_isaacsim.git" "145b6e0fb29386f6acc71a17741be94c9dff0e3e" "${target_root}/storm"
clone_checkout "https://github.com/isaac-sim/IsaacLab.git" "b5fa0eb031a2413c182eeb54fa3a9295e8fd867c" "${target_root}/IsaacLab"
clone_checkout "https://github.com/agimus-project/agimus-demos.git" "f1c5b37af3c24eab8b5be6c0f04e0f5aa8f50067" "${target_root}/ros2_ws/src/agimus-demos"
clone_checkout "https://github.com/agimus-project/franka_description.git" "535aa51107bf990abd92404d2517b09e4b6417a3" "${target_root}/ros2_ws/src/franka_description"
clone_checkout "https://github.com/agimus-project/franka_ros2.git" "64d972ea178f315bb1240fd4cda4f06900d5ef27" "${target_root}/ros2_ws/src/franka_ros2"
clone_checkout "https://github.com/agimus-project/libfranka.git" "fc51f939681577484034194f29048da92915a243" "${target_root}/ros2_ws/src/libfranka"
clone_checkout "https://github.com/loco-3d/linear-feedback-controller.git" "d591a1bc68ee702d405a99be159d0daf3aa9c674" "${target_root}/ros2_ws/src/linear-feedback-controller"
clone_checkout "https://github.com/loco-3d/linear-feedback-controller-msgs.git" "53307f5529cb6e55582a52da2cb750052a12f446" "${target_root}/ros2_ws/src/linear-feedback-controller-msgs"
clone_checkout "https://github.com/gursoyege/storm_agimus_bridge.git" "3388f174f450bd5729c10e01b9a28703e2fcf44b" "${target_root}/ros2_ws/src/storm_agimus_bridge"
clone_checkout "https://github.com/gursoyege/storm_pick_and_place_demo.git" "6210a4a5fd7cb050847a68b0ade880de19ebc348" "${target_root}/ros2_ws/src/storm_pick_place_demo"

# Keep only packages required by storm_pick_place_demo dependency closure.
keep_only "${target_root}/ros2_ws/src/agimus-demos" "agimus_demos_common"
keep_only "${target_root}/ros2_ws/src/franka_ros2" "franka_gazebo" "franka_gripper" "franka_hardware" "franka_msgs" "franka_robot_state_broadcaster" "franka_semantic_components"
keep_only "${target_root}/ros2_ws/src/franka_ros2/franka_gazebo" "franka_ign_ros2_control"

echo "Workspace populated at: ${target_root}"
echo "Next: docker build -t storm-stack ."
