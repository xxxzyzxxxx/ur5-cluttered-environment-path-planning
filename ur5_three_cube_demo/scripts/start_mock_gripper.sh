#!/usr/bin/env bash
set -e

for attempt in $(seq 1 30); do
  if ros2 param set \
      /controller_manager \
      gripper_controller.type \
      position_controllers/GripperActionController; then
    break
  fi

  if [ "$attempt" -eq 30 ]; then
    echo "Controller manager did not become available." >&2
    exit 1
  fi

  sleep 1
done

config_path="$(ros2 pkg prefix --share ur5_three_cube_demo)/config/gripper_controller.yaml"

exec ros2 run controller_manager spawner gripper_controller \
  --controller-manager /controller_manager \
  --param-file "$config_path"
