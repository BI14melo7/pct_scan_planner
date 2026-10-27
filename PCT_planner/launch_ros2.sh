#!/bin/bash
# Launches the PCT planner node + RViz2 together.
# Usage: ./launch_ros2.sh [--skip-tomo]

set -e

ROOT=$(cd "$(dirname "$0")"; pwd)
PYTHON="$ROOT/.venv/bin/python3"
CUDA_TOOLKIT_ROOT=/usr/local/cuda-12.8

if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: PCT Python environment not found at $ROOT/.venv"
    exit 1
fi

export CUDA_HOME="$CUDA_TOOLKIT_ROOT"
export CUDA_PATH="$CUDA_TOOLKIT_ROOT"
export CUDAToolkit_ROOT="$CUDA_TOOLKIT_ROOT"
export PATH="$CUDA_TOOLKIT_ROOT/bin:$PATH"
export LD_LIBRARY_PATH="$ROOT/planner/lib/3rdparty/gtsam-4.1.1/install/lib:$ROOT/planner/lib/build/src/common/smoothing:$CUDA_TOOLKIT_ROOT/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROOT/planner/lib:${PYTHONPATH:-}"

source /opt/ros/humble/setup.bash

# Launch the planner node in the background
"$PYTHON" "$ROOT/run_ros2_interactive.py" "$@" &
NODE_PID=$!

# Wait for the node to be ready (it logs "Interactive ROS2 mode" when done)
echo "Waiting for planner node to initialize..."
for i in $(seq 1 60); do
    sleep 1
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "ERROR: planner node exited unexpectedly."
        exit 1
    fi
done &
WATCHDOG=$!

# Open RViz2 with the PCT config
rviz2 -d "$ROOT/rsc/rviz/pct_ros2.rviz"

# Cleanup on RViz2 exit
kill $WATCHDOG 2>/dev/null || true
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
