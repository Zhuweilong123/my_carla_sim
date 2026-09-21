#!/usr/bin/env bash
set -Ee

# One-command WSL/WSLg launcher for the ROS 2 simulator.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-lyrical}"
scenario="${1:-obstacle}"
gui="${GUI:-true}"

case "${scenario}" in
    default|obstacle|three_lane|curve|figure_eight) ;;
    *)
        echo "Unknown scenario: ${scenario}" >&2
        echo "Choose: default, obstacle, three_lane, curve, figure_eight" >&2
        exit 2
        ;;
esac

ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -f "${ros_setup}" ]]; then
    echo "ROS 2 setup not found: ${ros_setup}" >&2
    echo "Set ROS_DISTRO to an installed distribution." >&2
    exit 1
fi

source "${ros_setup}"
set -u
cd "${repo_root}/.."

# colcon build (without --symlink-install) avoids the /mnt/d WSL filesystem
# cleanup issue encountered in this workspace.
if [[ "${SKIP_BUILD:-0}" != "1" || ! -f "${repo_root}/../install/setup.bash" ]]; then
    colcon build
fi

set +u
source "${repo_root}/../install/setup.bash"
set -u

export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-wayland}"
export SDL_RENDER_DRIVER="${SDL_RENDER_DRIVER:-software}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

launch_args=("gui:=${gui}" "scenario:=${scenario}")
if [[ -n "${NAMESPACE:-}" ]]; then
    launch_args+=("namespace:=${NAMESPACE}")
fi

exec ros2 launch lightweight_sim lightweight_sim.launch.py \
    "${launch_args[@]}" "${@:2}"
