#!/usr/bin/env bash
set -Ee

# Linux/WSL acceptance test for the ROS 2 migration.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${LIGHTWEIGHT_SIM_INSTALL:-}" ]]; then
    install_base="${LIGHTWEIGHT_SIM_INSTALL}"
elif [[ -f "${repo_root}/install/setup.bash" ]]; then
    install_base="${repo_root}/install"
else
    install_base="/tmp/lightweight_sim_install"
fi
ros_distro="${ROS_DISTRO:-lyrical}"
sim_namespace="${LIGHTWEIGHT_SIM_NAMESPACE:-}"
sim_namespace="${sim_namespace#/}"
sim_namespace="${sim_namespace%/}"
if [[ -n "${sim_namespace}" ]]; then
    launch_args=("namespace:=${sim_namespace}")
    topic_prefix="/${sim_namespace}"
else
    launch_args=()
    topic_prefix=""
fi
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

ros2 launch lightweight_sim lightweight_sim.launch.py "${launch_args[@]}" >"${launch_log}" 2>&1 &
launch_pid=$!

for _ in {1..20}; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic_prefix}/vehicle/state"; then
        break
    fi
    sleep 0.5
done

echo "--- topics ---"
ros2 topic list | sort

echo "--- time configuration ---"
planner_use_sim_time="$(ros2 param get "${topic_prefix}/planner_node" use_sim_time)"
controller_use_sim_time="$(ros2 param get "${topic_prefix}/controller_node" use_sim_time)"
simulator_use_sim_time="$(ros2 param get "${topic_prefix}/simulator_node" use_sim_time)"
printf '%s\n' "${planner_use_sim_time}" "${controller_use_sim_time}" "${simulator_use_sim_time}"
grep -q "Boolean value is: True" <<<"${planner_use_sim_time}"
grep -q "Boolean value is: True" <<<"${controller_use_sim_time}"
grep -q "Boolean value is: False" <<<"${simulator_use_sim_time}"

echo "--- reset before sampling ---"
ros2 service call "${topic_prefix}/sim/reset" std_srvs/srv/Empty '{}'

echo "--- vehicle state ---"
timeout 10s ros2 topic echo "${topic_prefix}/vehicle/state" --once >"${state_output}"
sed -n '1,12p' "${state_output}"

echo "--- planned path ---"
timeout 10s ros2 topic echo "${topic_prefix}/planned_path" --once >"${path_output}"
sed -n '1,12p' "${path_output}"

echo "--- simulation services ---"
ros2 service call "${topic_prefix}/sim/pause" std_srvs/srv/SetBool '{data: true}'
ros2 service call "${topic_prefix}/sim/step" std_srvs/srv/Trigger '{}'
ros2 service call "${topic_prefix}/sim/pause" std_srvs/srv/SetBool '{data: false}'

echo "ROS 2 smoke test passed"
