#!/usr/bin/env bash
# Run from the prepared ROS shell in the simulation package directory.
set -u
out=${1:?provide an output directory}
shift
if [ "$#" -eq 0 ]; then
  echo "Supply at least one map number (1-5)." >&2
  exit 2
fi
mkdir -p "$out" || exit 1
status=0
launcher=""
stop_world() {
  if [ -n "$launcher" ]; then
    kill -INT "$launcher" 2>/dev/null || true
    wait "$launcher" || true
    launcher=""
  fi
}
trap 'stop_world; exit 130' INT TERM
for n in "$@"; do
  case "$n" in [1-5]) ;; *) exit 2 ;; esac
  mkdir -p "$out/$n"
  roslaunch lab_vision_sim_task4 "task$n.launch" gui:=false interactive:=false >"$out/$n/launch.log" 2>&1 &
  launcher=$!
  for i in $(seq 1 120); do
    if ! kill -0 "$launcher" 2>/dev/null; then break; fi
    if rostopic list 2>/dev/null | grep -qx /usb_cam/image_raw; then break; fi
    sleep 1
  done
  if kill -0 "$launcher" 2>/dev/null; then
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 python scripts/run_map.py "$n" "$out/$n" >"$out/$n/mission.log" 2>&1 || status=1
  else
    status=1
  fi
  stop_world
  sleep 2
done
exit "$status"
