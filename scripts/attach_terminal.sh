#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="ros2-devcontainer"

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
	echo "Container '$CONTAINER_NAME' does not exist" >&2
	exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
	echo "Container '$CONTAINER_NAME' is not running" >&2
	exit 1
fi

docker container exec -it "$CONTAINER_NAME" /entrypoint.sh tmux
