#!/usr/bin/env bash
# PLC <-> robot PC over real PROFINET, on one machine, without sudo.
#
#   PLC namespace                         robot PC namespace
#   mw_plc_sim (this repo) ── pn-plc ═══ pn-dev ── mw_profinet_bridge (p-net)
#   192.168.0.1                  veth               + state machine (test tree)
#
# Two network namespaces joined by a veth pair stand in for two machines and a
# cable. Both ends need UDP 34964, which is why they cannot share one. An
# unprivileged user namespace gives the raw sockets and `ip link` rights both
# stacks need, so nothing here asks for root. The whole test is also its own
# PID namespace: when this script (its PID 1) exits, the kernel ends every
# process in it, so nothing outlives a run -- however it ends.
#
# One harness-only workaround: a veth reports 10 Gbit/s, which p-net v0.2.0
# rejects as "no fast port" at PrmEnd. fake_link_speed.c, preloaded into the
# bridge only, reports 1 Gbit/s for pn-dev. Real ports never need it.
#
# usage:  mw_plc_sim/loopback_test.sh [mw_plc_sim args]      e.g. --cycles 3
#         mw_plc_sim/loopback_test.sh                         interactive PLC HMI
#
# env:    ROS_SETUP  colon-separated setup.bash files for the robot side
#                    (default: ~/mw_ws/install/setup.bash:~/fuse_insetion_ws/install/setup.bash)
#         ROBOT      "sm" (default): bridge + fuse state machine running the test tree
#                    "bridge": bridge only
#         GSDML      robot PC GSDML (default: the one installed with mw_profinet_bridge)
#         SM_ARGS    extra --ros-args for the state machine
#         CAPTURE=1  tcpdump the robot side of the cable to <logs>/pn-dev.pcap
#         BRIDGE_WRAP  command prefix for the bridge, e.g. "strace -f -e trace=network -o /tmp/s.txt"
#         BRIDGE_PRELOAD  extra LD_PRELOAD for the bridge (e.g. libasan.so for a sanitizer build)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
: "${ROS_SETUP:=$HOME/mw_ws/install/setup.bash:$HOME/fuse_insetion_ws/install/setup.bash}"
: "${ROBOT:=sm}"
: "${SM_ARGS:=}"

if [ "${MW_LOOPBACK_INNER:-}" != 1 ]; then
  exec unshare --user --map-root-user --net --pid --fork --mount-proc \
    env MW_LOOPBACK_INNER=1 "$0" "$@"
fi

# ── inside the user namespace: this shell is the PLC side ──────────────────
source_ros() {
  set +u
  IFS=: read -ra files <<< "$ROS_SETUP"
  for f in "${files[@]}"; do [ -f "$f" ] && source "$f"; done
  set -u
}

if [ -z "${GSDML:-}" ]; then
  GSDML="$(source_ros; ls "$(ros2 pkg prefix mw_profinet_bridge)"/share/mw_profinet_bridge/gsdml/GSDML-*.xml | tail -1)"
fi

WORK="$(mktemp -d /tmp/mw-loopback.XXXX)"
gcc -shared -fPIC -o "$WORK/fake_link_speed.so" "$HERE/fake_link_speed.c" -ldl
ROBOT_PIDS=()
cleanup() {
  # SIGINT first so the bridge sends its last all-zero image and the state
  # machine stops its tree; anything left is ended with the PID namespace.
  for pid in "${ROBOT_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for _ in $(seq 30); do
    alive=0; for pid in "${ROBOT_PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.1
  done
  echo "robot-side logs: $WORK"
}

ip link set lo up
ip link add pn-plc type veth peer name pn-dev
# The robot PC's own network namespace. No --fork: unshare execs sleep in the
# new namespace, so $! is a process inside it (what `ip link set netns` and
# nsenter need) and killing it ends the namespace holder.
unshare --net sleep infinity </dev/null >/dev/null 2>&1 &
NS_PID=$!
trap cleanup EXIT
for _ in $(seq 50); do [ "$(readlink /proc/$NS_PID/ns/net)" != "$(readlink /proc/self/ns/net)" ] && break; sleep 0.05; done
[ "$(readlink /proc/$NS_PID/ns/net)" != "$(readlink /proc/self/ns/net)" ] || { echo "robot namespace not created" >&2; exit 1; }
ip link set pn-dev netns "$NS_PID"
ip addr add 192.168.0.1/24 dev pn-plc
ip link set pn-plc up

robot() { nsenter --net="/proc/$NS_PID/ns/net" "$@"; }
# Background a robot-side command. Not `robot ... &`: that backgrounds a
# subshell, and $! would be the subshell rather than the node.
robot_bg() {
  local log="$1"; shift
  nsenter --net="/proc/$NS_PID/ns/net" "$@" > "$log" 2>&1 &
  ROBOT_PIDS+=($!)
}
robot ip link set lo up
robot ip link set pn-dev up
if [ "${CAPTURE:-}" = 1 ]; then
  robot_bg "$WORK/tcpdump.log" tcpdump -Z root -i pn-dev -w "$WORK/pn-dev.pcap" -U
fi

# The bridge and the state machine are started directly (not via `ros2 run`),
# so the signals in cleanup() reach the nodes themselves.
robot_bg "$WORK/bridge.log" bash -c "
  $(declare -f source_ros); ROS_SETUP='$ROS_SETUP'; source_ros
  export ROS_HOME='$WORK/ros_home' ROS_LOG_DIR='$WORK/ros_log' ROS_LOCALHOST_ONLY=1
  export LD_PRELOAD='${BRIDGE_PRELOAD:+$BRIDGE_PRELOAD }$WORK/fake_link_speed.so' MW_FAKE_LINK_SPEED_IFACE=pn-dev
  exec ${BRIDGE_WRAP:-} \"\$(ros2 pkg prefix mw_profinet_bridge)/lib/mw_profinet_bridge/profinet_bridge\" \
    --ros-args -r __ns:=/plc -p transport:=pnet -p interface:=pn-dev
"

if [ "$ROBOT" = sm ]; then
  robot_bg "$WORK/state_machine.log" bash -c "
    $(declare -f source_ros); ROS_SETUP='$ROS_SETUP'; source_ros
    export ROS_HOME='$WORK/ros_home' ROS_LOG_DIR='$WORK/ros_log' ROS_LOCALHOST_ONLY=1
    exec \"\$(ros2 pkg prefix mw_fuse_insertion_sm)/lib/mw_fuse_insertion_sm/state_machine\" \
      --ros-args -p plc_single_cycle:=true \
      -p tree_xml:='$HERE/fixtures/robot_one_fasten.xml' -p init_tree_xml:='$HERE/fixtures/robot_init.xml' $SM_ARGS
  "
fi

echo "robot side starting (logs in $WORK); PLC on pn-plc, GSDML $GSDML"
set +e
"$REPO/.venv/bin/python" -m mw_plc_sim --iface pn-plc --gsdml "$GSDML" "$@"
rc=$?
set -e
exit $rc
