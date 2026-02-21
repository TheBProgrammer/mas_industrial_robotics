docker run --rm -it \
  -v /tmp/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
  -v /run/user/${UID}/wayland-0:/run/user/${UID}/wayland-0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/dri:/dev/dri \
  --device=/dev/kfd:/dev/kfd \
  -e DISPLAY=${DISPLAY} \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=/tmp \
  --group-add=$(getent group video | cut -d: -f3) \
  skratch_sim:latest bash
