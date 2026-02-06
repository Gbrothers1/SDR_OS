#!/usr/bin/env bash
set -eo pipefail

# Source ROS setup
if ! source /opt/ros/jazzy/setup.bash 2>/dev/null; then
  echo "Warning: Failed to source ROS setup. Continuing anyway..." >&2
fi

ROS_ENABLE_ROSBRIDGE="${ROS_ENABLE_ROSBRIDGE:-1}"
ROS_NODE_COMMANDS="${ROS_NODE_COMMANDS:-}"

pids=()

if [[ "${ROS_ENABLE_ROSBRIDGE}" == "1" ]]; then
  echo "Starting rosbridge_server..." >&2
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
  pids+=("$!")
fi

if [[ -n "${ROS_NODE_COMMANDS}" ]]; then
  IFS=';' read -r -a commands <<< "${ROS_NODE_COMMANDS}"
  for cmd in "${commands[@]}"; do
    cmd_trimmed="${cmd## }"
    cmd_trimmed="${cmd_trimmed%% }"
    if [[ -n "${cmd_trimmed}" ]]; then
      bash -lc "${cmd_trimmed}" &
      pids+=("$!")
    fi
  done
fi

if [[ ${#pids[@]} -eq 0 ]]; then
  echo "No ROS processes configured. Set ROS_NODE_COMMANDS or enable rosbridge." >&2
  echo "Container will stay alive. Use 'docker exec' to get a shell." >&2
  tail -f /dev/null
else
  echo "Started ${#pids[@]} background process(es). Waiting..." >&2
  wait -n || true
  echo "One of the background processes exited. Container will stay alive." >&2
  tail -f /dev/null
fi
