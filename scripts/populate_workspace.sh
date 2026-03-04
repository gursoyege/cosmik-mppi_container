#!/usr/bin/env bash
set -euo pipefail

target_root="${1:-$(pwd)/workspace}"

clone_checkout_sha() {
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

clone_latest_branch() {
  local repo_url="$1"
  local branch="$2"     # e.g. main
  local dst_dir="$3"

  if [[ -d "${dst_dir}/.git" ]]; then
    git -C "${dst_dir}" fetch --prune --tags origin
  else
    rm -rf "${dst_dir}"
    git clone --branch "${branch}" --single-branch "${repo_url}" "${dst_dir}"
  fi

  git -C "${dst_dir}" checkout --force "${branch}"
  git -C "${dst_dir}" reset --hard "origin/${branch}"
  git -C "${dst_dir}" submodule update --init --recursive
}

mkdir -p "${target_root}" "${target_root}/ros2_ws/src"

clone_latest_branch "https://github.com/gursoyege/storm_isaacsim.git" "main" "${target_root}/storm"
clone_latest_branch "https://github.com/gursoyege/storm_agimus_bridge.git" "master" "${target_root}/ros2_ws/src/storm_agimus_bridge"
clone_latest_branch "https://github.com/gursoyege/storm_pick_and_place_demo.git" "master" "${target_root}/ros2_ws/src/storm_pick_place_demo"
clone_latest_branch "https://github.com/gursoyege/agimus_demos_common.git" "master" "${target_root}/ros2_ws/src/agimus_demos_common"

clone_checkout_sha "https://github.com/isaac-sim/IsaacLab.git" "b5fa0eb031a2413c182eeb54fa3a9295e8fd867c" "${target_root}/IsaacLab"
clone_checkout_sha "https://github.com/agimus-project/franka_description.git" "535aa51107bf990abd92404d2517b09e4b6417a3" "${target_root}/ros2_ws/src/franka_description"
clone_checkout_sha "https://github.com/agimus-project/franka_ros2.git" "64d972ea178f315bb1240fd4cda4f06900d5ef27" "${target_root}/ros2_ws/src/franka_ros2"
clone_checkout_sha "https://github.com/agimus-project/libfranka.git" "fc51f939681577484034194f29048da92915a243" "${target_root}/ros2_ws/src/libfranka"
clone_checkout_sha "https://github.com/loco-3d/linear-feedback-controller.git" "d591a1bc68ee702d405a99be159d0daf3aa9c674" "${target_root}/ros2_ws/src/linear-feedback-controller"
clone_checkout_sha "https://github.com/loco-3d/linear-feedback-controller-msgs.git" "53307f5529cb6e55582a52da2cb750052a12f446" "${target_root}/ros2_ws/src/linear-feedback-controller-msgs"

echo "Workspace populated at: ${target_root}"
echo "Next: docker build -t storm-stack ."