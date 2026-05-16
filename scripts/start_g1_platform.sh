#!/usr/bin/env bash
# start_g1_platform.sh — one-command "G1 active on a level tilting platform".
#
# Brings up:
#   • unitree_mujoco with scene_platform.xml, platform_mode=remote (level at 0,0)
#   • g1_ctrl  (the on-robot FSM + RL controller from unitree_rl_mjlab)
#   • auto-engages the FixStand → Velocity FSM transitions
#
# When ready, the G1 is standing on the (flat) platform in Velocity mode, ready
# to receive walk commands. To switch the platform to "boat" or drive it
# manually, run keyboard_platform / pulse_platform from another terminal — the
# sim is already in "remote" mode and listens on rt/platform_cmd.
#
# Press Ctrl+C to stop everything and restore the original config.yaml.

set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
UM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RL_DEPLOY="${RL_DEPLOY:-$HOME/work/unitree_rl_mjlab/deploy/robots/g1}"

SIM_BIN="$UM_DIR/simulate/build/unitree_mujoco"
CTRL_BIN="$RL_DEPLOY/build/g1_ctrl"
PULSE="$UM_DIR/tools/keyboard_gamepad/build/pulse_gamepad"
CFG="$UM_DIR/simulate/config.yaml"
CFG_BAK="$UM_DIR/simulate/config.yaml.bak.start_g1_platform"
SIM_LOG="${SIM_LOG:-/tmp/sim.log}"
CTRL_LOG="${CTRL_LOG:-/tmp/g1ctrl.log}"
IFACE="${IFACE:-lo}"

for f in "$SIM_BIN" "$CTRL_BIN" "$PULSE"; do
    [[ -x "$f" ]] || { echo "missing binary: $f" >&2; exit 1; }
done

SIM_PID=""
CTRL_PID=""

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "--- stopping ---"
    [[ -n "$CTRL_PID" ]] && kill "$CTRL_PID" 2>/dev/null
    [[ -n "$SIM_PID"  ]] && kill "$SIM_PID"  2>/dev/null
    sleep 0.5
    [[ -n "$CTRL_PID" ]] && kill -9 "$CTRL_PID" 2>/dev/null
    [[ -n "$SIM_PID"  ]] && kill -9 "$SIM_PID"  2>/dev/null
    if [[ -f "$CFG_BAK" ]]; then
        mv -f "$CFG_BAK" "$CFG"
        echo "restored $CFG"
    fi
}
trap cleanup INT TERM EXIT

echo "--- configuring $CFG (backup at $CFG_BAK) ---"
cp -f "$CFG" "$CFG_BAK"
# Use [^"]* (not .*) so the regex stops at the closing quote and doesn't swallow
# the trailing inline comment, which would break YAML parsing.
# Bump platform_kp so the plate stays level even with an off-balance G1 standing
# (or briefly collapsing) on top — the default 800 lets the robot's weight tilt
# it visibly while the FSM is still in Passive.
sed -i \
    -e 's|^robot: "[^"]*"|robot: "g1"|' \
    -e 's|^robot_scene: "[^"]*"|robot_scene: "scene_platform.xml"|' \
    -e 's|^enable_elastic_band: [0-9]*|enable_elastic_band: 0|' \
    -e 's|^platform_mode: "[^"]*"|platform_mode: "remote"|' \
    -e 's|^platform_kp: [0-9.]*|platform_kp: 3000.0|' \
    -e 's|^platform_kd: [0-9.]*|platform_kd: 200.0|' \
    "$CFG"
grep -E '^(robot|robot_scene|enable_elastic_band|platform_mode):' "$CFG" | sed 's/^/  /'

: > "$SIM_LOG"
: > "$CTRL_LOG"

# Make sure the platform target starts at 0,0 (level) by publishing a zero
# command in the background once the sim has come up.

: "${DISPLAY:=:100}"
export DISPLAY
echo "--- starting sim (DISPLAY=$DISPLAY) ---"
"$SIM_BIN" >"$SIM_LOG" 2>&1 &
SIM_PID=$!

echo "--- starting g1_ctrl (network=$IFACE) ---"
"$CTRL_BIN" --network="$IFACE" >"$CTRL_LOG" 2>&1 &
CTRL_PID=$!

wait_for_log() {
    local logfile="$1" pattern="$2" timeout="${3:-30}"
    local elapsed=0
    while ! grep -q -- "$pattern" "$logfile" 2>/dev/null; do
        if ! kill -0 "$SIM_PID" 2>/dev/null; then echo "  sim died"; return 1; fi
        if ! kill -0 "$CTRL_PID" 2>/dev/null; then echo "  ctrl died"; return 1; fi
        sleep 0.3
        elapsed=$((elapsed + 1))
        if (( elapsed > timeout * 3 )); then
            echo "  timeout waiting for: $pattern"
            return 2
        fi
    done
}

echo "--- waiting for g1_ctrl FSM ---"
if ! wait_for_log "$CTRL_LOG" 'FSM: Start Passive' 20; then
    echo "g1_ctrl never reached FSM (see $CTRL_LOG)"; exit 1
fi
echo "  ✓ FSM running in Passive"

# Pre-seed a level platform target (rt/platform_cmd needs at least one msg).
"$PULSE" 0x0000 0.2 "$IFACE" >/dev/null 2>&1 || true

echo "--- macro 1: Passive -> FixStand ---"
"$PULSE" 0x1020 0.6 "$IFACE" >/dev/null 2>&1
if ! wait_for_log "$CTRL_LOG" 'Passive to FixStand' 10; then
    echo "FixStand transition did not occur — retrying once"
    "$PULSE" 0x1020 0.8 "$IFACE" >/dev/null 2>&1
    wait_for_log "$CTRL_LOG" 'Passive to FixStand' 10 || { echo "still no FixStand — abort"; exit 1; }
fi
echo "  ✓ FixStand active"
sleep 1

echo "--- macro 2: FixStand -> Velocity ---"
"$PULSE" 0x0110 0.6 "$IFACE" >/dev/null 2>&1
if wait_for_log "$CTRL_LOG" 'FixStand to Velocity' 6; then
    echo "  ✓ Velocity (RL walking) active"
else
    echo "  ! Velocity transition didn't fire — robot stays in FixStand (still active)"
fi

echo
echo "=========================================="
echo " G1 is now active on the level platform."
echo "=========================================="
echo "  sim:   pid=$SIM_PID  log=$SIM_LOG"
echo "  ctrl:  pid=$CTRL_PID log=$CTRL_LOG"
echo
echo " From other terminals you can now:"
echo "   • Drive the platform:   $UM_DIR/tools/platform/build/keyboard_platform"
echo "   • Pulse a tilt:         $UM_DIR/tools/platform/build/pulse_platform 10 0 30 2 $IFACE"
echo "   • Drive the robot:      $UM_DIR/tools/keyboard_gamepad/build/keyboard_gamepad"
echo "   • Switch to boat mode:  edit config.yaml platform_mode: \"boat\", restart"
echo
echo " Ctrl+C here stops everything and restores config.yaml."
echo

# Wait for either child to exit (or signal), then cleanup via trap.
while kill -0 "$SIM_PID" 2>/dev/null && kill -0 "$CTRL_PID" 2>/dev/null; do
    sleep 1
done
echo "--- one of the child processes exited ---"
