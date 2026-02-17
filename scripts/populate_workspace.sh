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
    git -C "${dst_dir}" fetch --prune --tags origin
  fi

  git -C "${dst_dir}" checkout --force "${commit_sha}"
  git -C "${dst_dir}" submodule update --init --recursive
}

# For repos where you don't pin a commit: keep them updated and checked out to the remote default branch.
clone_default_branch() {
  local repo_url="$1"
  local dst_dir="$2"

  if [[ -d "${dst_dir}/.git" ]]; then
    git -C "${dst_dir}" remote set-url origin "${repo_url}"
    git -C "${dst_dir}" fetch --prune --tags origin
  else
    rm -rf "${dst_dir}"
    git clone "${repo_url}" "${dst_dir}"
    git -C "${dst_dir}" fetch --prune --tags origin
  fi

  # Detach at origin/HEAD (remote's default branch)
  git -C "${dst_dir}" checkout --force "$(git -C "${dst_dir}" symbolic-ref -q --short refs/remotes/origin/HEAD || echo origin/HEAD)"
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
clone_checkout "https://github.com/agimus-project/franka_description.git" "535aa51107bf990abd92404d2517b09e4b6417a3" "${target_root}/ros2_ws/src/franka_description"
clone_checkout "https://github.com/agimus-project/libfranka.git" "fc51f939681577484034194f29048da92915a243" "${target_root}/ros2_ws/src/libfranka"
clone_checkout "https://github.com/loco-3d/linear-feedback-controller.git" "d591a1bc68ee702d405a99be159d0daf3aa9c674" "${target_root}/ros2_ws/src/linear-feedback-controller"
clone_checkout "https://github.com/loco-3d/linear-feedback-controller-msgs.git" "53307f5529cb6e55582a52da2cb750052a12f446" "${target_root}/ros2_ws/src/linear-feedback-controller-msgs"

# These are ROS packages too -> put them under ros2_ws/src (and make it idempotent).
clone_default_branch "https://github.com/gursoyege/agimus_franka_ros2.git" "${target_root}/ros2_ws/src/franka_ros2"
clone_default_branch "https://github.com/gursoyege/agimus_demos_common.git" "${target_root}/ros2_ws/src/agimus-demos"
clone_default_branch "https://github.com/gursoyege/storm_agimus_bridge.git" "${target_root}/ros2_ws/src/storm_agimus_bridge"
clone_default_branch "https://github.com/gursoyege/storm_pick_and_place_demo.git" "${target_root}/ros2_ws/src/storm_pick_and_place_demo"

echo "Workspace populated at: ${target_root}"
echo "Next: docker build -t storm-stack ."
