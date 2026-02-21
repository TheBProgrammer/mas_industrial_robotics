#!/bin/bash
set -e

source "/opt/ros/humble/setup.bash"

if [ -f "$HOME/skratch_ws/install/setup.bash" ]; then
  source "$HOME/skratch_ws/install/setup.bash"
fi

exec "$@"
