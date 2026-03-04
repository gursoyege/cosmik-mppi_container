#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/.." && pwd)"
compose_file="${root_dir}/docker/docker-compose.yml"

image_name="${IMAGE_NAME:-cosmik-mppi}"
service_name="${SERVICE_NAME:-cosmik-mppi}"

if docker compose version >/dev/null 2>&1; then
  compose_cmd=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  compose_cmd=(docker-compose)
else
  echo "Neither 'docker compose' nor 'docker-compose' is available."
  exit 1
fi

mapfile -t matching_containers < <(
  docker ps -a --filter "ancestor=${image_name}" \
    --format '{{.ID}}|{{.Names}}|{{.State}}|{{.Status}}'
)

if (( ${#matching_containers[@]} == 0 )); then
  if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
    echo "Image '${image_name}' does not exist."
    echo "Build it first with: bash ${script_dir}/build_all.sh"
    exit 1
  fi

  echo "No existing container found for image '${image_name}'."
  echo "Creating and starting one with compose service '${service_name}'..."
  "${compose_cmd[@]}" -f "${compose_file}" up -d "${service_name}"

  mapfile -t matching_containers < <(
    docker ps -a --filter "ancestor=${image_name}" \
      --format '{{.ID}}|{{.Names}}|{{.State}}|{{.Status}}'
  )

  if (( ${#matching_containers[@]} == 0 )); then
    echo "Could not find a container from image '${image_name}' after compose up."
    exit 1
  fi
fi

IFS='|' read -r selected_id selected_name selected_state _ <<< "${matching_containers[0]}"

if (( ${#matching_containers[@]} > 1 )); then
  echo "Warning: found ${#matching_containers[@]} containers from image '${image_name}'. Using the newest one: ${selected_name}."
  for container_info in "${matching_containers[@]}"; do
    IFS='|' read -r id name state status <<< "${container_info}"
    echo "  - ${name} (${id:0:12}) ${status}"
  done
fi

if [[ "${selected_state}" != "running" ]]; then
  echo "Starting container '${selected_name}' (${selected_id:0:12})..."
  docker start "${selected_id}" >/dev/null
fi

echo "Opening bash in '${selected_name}'..."
docker exec -it "${selected_id}" bash
