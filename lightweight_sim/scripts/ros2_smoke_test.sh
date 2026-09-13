#!/usr/bin/env bash
set -Ee

# Linux/WSL acceptance test for the ROS 2 migration.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_base="${LIGHTWEIGHT_SIM_INSTALL:-/tmp/lightweight_sim_install}"
ros_distro="${ROS_DISTRO:-lyrical}"
launch_log="$(mktemp /tmp/lightweight_sim_smoke.XXXXXX.log)"
state_output="$(mktemp /tmp/lightweight_sim_state.XXXXXX.txt)"
path_output="$(mktemp /tmp/lightweight_sim_path.XXXXXX.txt)"
launch_pid=""

cleanup() {
    set +e
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT "${launch_pid}" 2>/dev/null
        sleep 2
        kill -TERM "${launch_pid}" 2>/dev/null
        wait "${launch_pid}" 2>/dev/null
    fi
    rm -f "${launch_log}" "${state_output}" "${path_output}"
}
trap cleanup EXIT

source "/opt/ros/${ros_distro}/setup.bash"
source "${install_base}/setup.bash"
set -u
cd "${repo_root}"

ros2 launch lightweight_sim lightweight_sim.launch.py >"${launch_log}" 2>&1 &
launch_pid=$!

for _ in {1..20}; do
    if ros2 topic list 2>/dev/null | grep -qx "/vehicle/state"; then
        break
    fi
    sleep 0.5
done

echo "--- topics ---"
ros2 topic list | sort

echo "--- vehicle state ---"
timeout 10s ros2 topic echo /vehicle/state --once >"${state_output}"
sed -n '1,12p' "${state_output}"

echo "--- planned path ---"
timeout 10s ros2 topic echo /planned_path --once >"${path_output}"
sed -n '1,12p' "${path_output}"

echo "--- simulation services ---"
ros2 service call /sim/reset std_srvs/srv/Empty '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/step std_srvs/srv/Trigger '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'

echo "ROS 2 smoke test passed"
