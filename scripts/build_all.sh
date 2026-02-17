#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/.." && pwd)"
docker_dir="${root_dir}/docker"
workspace_dir="${root_dir}/workspace"

image_name="${IMAGE_NAME:-cosmik-mppi}"
service_name="${SERVICE_NAME:-cosmik-mppi}"

if [[ ! -x "${script_dir}/populate_workspace.sh" ]]; then
  echo "Missing executable: ${script_dir}/populate_workspace.sh"
  exit 1
fi

if [[ ! -f "${docker_dir}/Dockerfile" ]]; then
  echo "Missing Dockerfile: ${docker_dir}/Dockerfile"
  exit 1
fi

if [[ ! -f "${docker_dir}/docker-compose.yml" ]]; then
  echo "Missing compose file: ${docker_dir}/docker-compose.yml"
  exit 1
fi

"${script_dir}/populate_workspace.sh" "${workspace_dir}"
docker build -t "${image_name}" -f "${docker_dir}/Dockerfile" "${root_dir}"

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
  if xhost +local:docker >/dev/null 2>&1; then
    trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
  else
    echo "Warning: could not run 'xhost +local:docker' (DISPLAY=${DISPLAY}). Continuing without X11 ACL changes."
  fi
else
  echo "DISPLAY is not set or xhost is unavailable; skipping X11 ACL setup."
fi

if docker compose version >/dev/null 2>&1; then
  compose_cmd=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  compose_cmd=(docker-compose)
else
  echo "Neither 'docker compose' nor 'docker-compose' is available."
  exit 1
fi

if [[ "$#" -gt 0 ]]; then
  container_cmd=("$@")
else
  container_cmd=("bash")
fi

"${compose_cmd[@]}" -f "${docker_dir}/docker-compose.yml" run "${service_name}" \
  -lc '/usr/local/bin/run_inside_docker.sh && exec "$@"' _ "${container_cmd[@]}"
