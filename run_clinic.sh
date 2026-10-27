#!/bin/bash
# End-to-end script: tomography + planning for clinic.pcd
#
# Usage:
#   ./run_clinic.sh                          # standalone (no ROS, default start/end)
#   ./run_clinic.sh 0 0 10 5                 # standalone, custom start/end
#   ./run_clinic.sh --ros2                   # ROS2 mode (publishes to RViz2)
#   ./run_clinic.sh --ros2 0 0 10 5         # ROS2 mode, custom start/end

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

ROS2_MODE=0
if [[ "$1" == "--ros2" ]]; then
    ROS2_MODE=1
    shift
    source /opt/ros/humble/setup.bash 2>/dev/null || true
fi

START_X=${1:-0.0}
START_Y=${2:-0.0}
END_X=${3:-10.0}
END_Y=${4:-5.0}

echo "=== Step 1: Tomography (clinic.pcd -> clinic.pickle) ==="
cd "$ROOT/tomography/scripts"
if [[ "$ROS2_MODE" == "1" ]]; then
    "$PYTHON" run_standalone.py --scene Clinic  # generate tomogram first
else
    "$PYTHON" run_standalone.py --scene Clinic
fi

echo ""
echo "=== Step 2: Planning (start=[${START_X}, ${START_Y}] -> end=[${END_X}, ${END_Y}]) ==="
cd "$ROOT/planner/scripts"
if [[ "$ROS2_MODE" == "1" ]]; then
    echo "(ROS2 mode: publishing path to /pct_path topic)"
    "$PYTHON" plan_ros2.py --tomo clinic \
        --start "$START_X" "$START_Y" \
        --end   "$END_X"   "$END_Y"
else
    "$PYTHON" plan_standalone.py --tomo clinic \
        --start "$START_X" "$START_Y" \
        --end   "$END_X"   "$END_Y"
fi

echo ""
echo "=== Done! Trajectory saved to rsc/clinic_traj.npy ==="
