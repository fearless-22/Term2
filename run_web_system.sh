#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$ROOT_DIR/embodied-sim-lite"
WS_DIR="$ROOT_DIR/ros2_slam_ws"

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
START_RVIZ=1
OPEN_BROWSER=1
REBUILD=0
FRESH_START=0

usage() {
  cat <<'EOF'
Usage: ./run_web_system.sh [options]

Options:
  --rebuild             Rebuild custom_slam before launching.
  --fresh               Stop existing project processes before launching.
  --no-rviz            Do not start RViz2.
  --no-browser         Do not open http://localhost:8000 automatically.
  --ros-distro NAME    ROS 2 distro to source. Default: $ROS_DISTRO or humble.
  -h, --help           Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rebuild)
      REBUILD=1
      FRESH_START=1
      shift
      ;;
    --fresh)
      FRESH_START=1
      shift
      ;;
    --no-rviz)
      START_RVIZ=0
      shift
      ;;
    --no-browser)
      OPEN_BROWSER=0
      shift
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      if [[ -z "$ROS_DISTRO_NAME" ]]; then
        echo "[ERROR] --ros-distro requires a value." >&2
        exit 2
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT_DIR/logs/web_system_${STAMP}"

PIDS=()
NAMES=()
REQUIRED_PIDS=()
REQUIRED_NAMES=()
WEB_STARTED=0

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -f "$path" ]]; then
    echo "[ERROR] Missing ${label}: $path" >&2
    exit 1
  fi
}

source_setup_file() {
  local setup_file="$1"
  local shell_options="$-"

  set +u
  source "$setup_file"
  if [[ "$shell_options" == *u* ]]; then
    set -u
  else
    set +u
  fi
}

source_ros() {
  source_setup_file "$ROS_SETUP"
}

source_workspace() {
  source_setup_file "$WS_SETUP"
}

source_workspace_if_present() {
  if [[ -f "$WS_SETUP" ]]; then
    source_workspace
  fi
}

web_is_ready() {
  python3 -c "import socket; s = socket.create_connection(('127.0.0.1', 8000), 1); s.close()" >/dev/null 2>&1
}

wait_for_web() {
  local web_pid="${1:-}"
  local max_wait_sec=30
  for elapsed in $(seq 1 "$max_wait_sec"); do
    if web_is_ready; then
      return 0
    fi
    if [[ -n "$web_pid" ]] && ! kill -0 "$web_pid" >/dev/null 2>&1; then
      return 1
    fi
    if [[ -n "$web_pid" && "$elapsed" -ge 5 ]]; then
      echo "[WARN] Web port check did not pass, but the Web process is still running; continuing."
      return 0
    fi
    sleep 1
  done
  return 1
}

register_pid() {
  local name="$1"
  local pid="$2"
  local required="${3:-1}"
  PIDS+=("$pid")
  NAMES+=("$name")
  if [[ "$required" == "1" ]]; then
    REQUIRED_PIDS+=("$pid")
    REQUIRED_NAMES+=("$name")
  fi
}

stop_existing_processes() {
  echo "[INFO] Stopping existing project processes..."
  pkill -INT -f "ProductV1.0.py" >/dev/null 2>&1 || true
  pkill -INT -f "sim_ros2_bridgeV1.0.py" >/dev/null 2>&1 || true
  pkill -INT -f "ros2 launch custom_slam system_launch.py" >/dev/null 2>&1 || true
  sleep 2
  pkill -TERM -f "ProductV1.0.py" >/dev/null 2>&1 || true
  pkill -TERM -f "sim_ros2_bridgeV1.0.py" >/dev/null 2>&1 || true
  pkill -TERM -f "ros2 launch custom_slam system_launch.py" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_code=$?
  trap - INT TERM EXIT

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo
    echo "[INFO] Stopping launched processes..."
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        kill -INT "$pid" >/dev/null 2>&1 || true
      fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        kill -TERM "$pid" >/dev/null 2>&1 || true
      fi
    done
    wait >/dev/null 2>&1 || true
  fi

  echo "[INFO] Logs: $LOG_DIR"
  exit "$exit_code"
}

start_web() {
  if web_is_ready; then
    echo "[INFO] Web simulation is already reachable at http://localhost:8000"
    return
  fi

  echo "[INFO] Starting Web simulation..."
  (
    cd "$SIM_DIR"
    exec python3 ProductV1.0.py
  ) >"$LOG_DIR/web.log" 2>&1 &
  local pid=$!
  WEB_STARTED=1
  register_pid "web" "$pid" 1

  if ! wait_for_web "$pid"; then
    echo "[ERROR] Web simulation did not become ready within 30 seconds." >&2
    echo "[ERROR] Check: $LOG_DIR/web.log" >&2
    exit 1
  fi
}

start_bridge() {
  echo "[INFO] Starting ROS2 bridge..."
  (
    source_ros
    source_workspace_if_present
    cd "$SIM_DIR"
    exec python3 sim_ros2_bridgeV1.0.py
  ) >"$LOG_DIR/bridge.log" 2>&1 &
  register_pid "bridge" "$!" 1
}

start_system() {
  echo "[INFO] Starting integrated ROS2 system..."
  (
    source_ros
    source_workspace
    cd "$WS_DIR"
    exec ros2 launch custom_slam system_launch.py
  ) >"$LOG_DIR/system.log" 2>&1 &
  register_pid "system" "$!" 1
}

start_rviz() {
  if [[ "$START_RVIZ" != "1" ]]; then
    return
  fi
  if ! (source_ros && command -v rviz2 >/dev/null 2>&1); then
    echo "[WARN] rviz2 command not found; skipping RViz2."
    return
  fi
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "[WARN] No display session detected; skipping RViz2."
    return
  fi

  echo "[INFO] Starting RViz2..."
  (
    source_ros
    source_workspace
    exec rviz2
  ) >"$LOG_DIR/rviz2.log" 2>&1 &
  register_pid "rviz2" "$!" 0
}

open_browser() {
  if [[ "$OPEN_BROWSER" != "1" ]]; then
    return
  fi
  if ! command -v xdg-open >/dev/null 2>&1; then
    return
  fi
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    return
  fi

  xdg-open http://localhost:8000 >/dev/null 2>&1 || true
}

monitor_required_processes() {
  echo "[INFO] All launch commands issued."
  echo "[INFO] Web:  http://localhost:8000"
  echo "[INFO] Logs: $LOG_DIR"
  echo "[INFO] Press Ctrl+C to stop everything started by this script."

  while true; do
    for i in "${!REQUIRED_PIDS[@]}"; do
      local pid="${REQUIRED_PIDS[$i]}"
      if ! kill -0 "$pid" >/dev/null 2>&1; then
        echo "[ERROR] Required process exited: ${REQUIRED_NAMES[$i]}" >&2
        exit 1
      fi
    done
    sleep 2
  done
}

main() {
  require_file "$SIM_DIR/ProductV1.0.py" "Web simulation"
  require_file "$SIM_DIR/sim_ros2_bridgeV1.0.py" "ROS2 bridge"
  require_file "$ROS_SETUP" "ROS setup file"
  mkdir -p "$LOG_DIR"
  export ROS_LOG_DIR="$LOG_DIR/ros"
  mkdir -p "$ROS_LOG_DIR"

  trap cleanup INT TERM EXIT

  if [[ "$FRESH_START" == "1" ]]; then
    stop_existing_processes
  fi

  if [[ "$REBUILD" == "1" || ! -f "$WS_SETUP" ]]; then
    echo "[INFO] Building custom_slam..."
    (
      source_ros
      cd "$WS_DIR"
      colcon build --symlink-install --packages-select custom_slam
    ) >"$LOG_DIR/build.log" 2>&1
  fi

  require_file "$WS_SETUP" "workspace setup file"

  start_web
  open_browser
  start_bridge
  sleep 2
  start_system
  sleep 4
  start_rviz
  monitor_required_processes
}

main "$@"
