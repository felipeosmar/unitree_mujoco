# Remote G1 simulation with the real on-robot controller

Run the Unitree G1 simulation **remotely** (over SSH) and drive it with the same
RL controller binary (`g1_ctrl` from `unitree_rl_mjlab/deploy`) that runs on the
physical robot — including the FSM (Passive → FixStand → Velocity / Mimic) — all
without a physical gamepad.

## TL;DR — how to run

Three SSH terminals. Each block can be copy-pasted as-is.

**Terminal 1 — simulator** (the MuJoCo window opens inside the Xpra session):

```bash
cd /home/ctrob/work/unitree_mujoco/simulate/build
DISPLAY=:100 ./unitree_mujoco
```

**Terminal 2 — on-robot controller** (FSM + RL policies):

```bash
cd /home/ctrob/work/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=lo
```

**Terminal 3 — virtual gamepad** (keyboard → DDS WirelessController):

```bash
/home/ctrob/work/unitree_mujoco/tools/keyboard_gamepad/build/keyboard_gamepad
```

Press `1` to stand (Passive → FixStand), `2` to walk (FixStand → Velocity), then
use `i / k / j / l` to bump the left stick (forward/back/turn).

Non-interactive equivalent (drives FSM from a shell script):

```bash
TOOL=/home/ctrob/work/unitree_mujoco/tools/keyboard_gamepad/build/pulse_gamepad
$TOOL 0x1020 0.5 lo   # L2 + up   → Passive  → FixStand
sleep 3
$TOOL 0x0110 0.5 lo   # R2 + A    → FixStand → Velocity
```

## How the pieces fit together

```
                  ┌──────────────────────┐         DDS topics on domain 0, iface lo
                  │ MuJoCo unitree_mujoco│ ───── rt/lowstate (motor q/dq/τ, IMU,
                  │  - physics 1 kHz     │         + wireless_remote bytes)
                  │  - G1Bridge          │ ───── rt/secondary_imu
                  │  - DDSJoystick (new) │ ◄──── rt/lowcmd
                  └──────────────────────┘ ◄──── rt/wirelesscontroller
                            ▲                       ▲
                            │ MuJoCo display        │ keyboard → bits
                            │ (X11 via Xpra)        │
                  ┌─────────┴────────────┐  ┌───────┴──────────────┐
                  │ Windows Xpra client  │  │ keyboard_gamepad     │
                  └──────────────────────┘  │  / pulse_gamepad     │
                                            └──────────────────────┘
                            ▲
                            │ DDS via rt/lowstate, rt/lowcmd
                            │
                  ┌─────────┴────────────┐
                  │ g1_ctrl              │
                  │ (FSM + ONNX policies)│
                  │  Passive / FixStand /│
                  │  Velocity / Mimic    │
                  └──────────────────────┘
```

Key point: `g1_ctrl` is the **same** C++ binary you would run on the real G1's
onboard computer. It speaks unmodified Unitree DDS (`rt/lowcmd` / `rt/lowstate`
with the `hg` IDL). The simulator implements that contract via its `G1Bridge`,
so the controller can't tell the difference.

The controller's FSM transitions are gated by the gamepad state embedded in
`LowState_::wireless_remote` (40 bytes). On the real robot those bytes come
from the Unitree wireless remote; here they come from `DDSJoystick`, a new
class added to the bridge that receives `WirelessController_` over DDS.

## What was set up on this machine

| Component | Path | Notes |
|---|---|---|
| Simulator source | `/home/ctrob/work/unitree_mujoco/` | This repo. Patched (see below). |
| Unitree SDK source | `/home/ctrob/work/unitree_sdk2/` | Cloned for examples. |
| SDK installed | `/opt/unitree_robotics/` | `libunitree_sdk2.a`, `ddsc`, `ddscxx`, headers. |
| G1 controller source | `/home/ctrob/work/unitree_rl_mjlab/` | Already on the machine; G1 deploy compiled. |
| Xpra (remote X) | apt package `xpra 3.1.5` | session on display `:100`, TCP 14500 |

### Patches applied to `unitree_mujoco`

1. **New** `simulate/src/dds_joystick.h` — `DDSJoystick : UnitreeJoystick` that
   subscribes to `rt/wirelesscontroller`. Disables `Axis` smoothing on L2/R2/
   sticks because the input is already digital/precise.
2. **`simulate/src/unitree_sdk2_bridge.h`** — adds a `joystick_type: "dds"`
   branch; skips the bridge's own `rt/wirelesscontroller` publish to avoid a
   feedback loop with the external publisher.
3. **`simulate/config.yaml`** — switched to G1:
   ```yaml
   robot: "g1"
   robot_scene: "scene_29dof.xml"
   domain_id: 0
   interface: "lo"
   use_joystick: 1
   joystick_type: "dds"
   ```

### New tools

`unitree_mujoco/tools/keyboard_gamepad/` — standalone CMake project that
builds two binaries:

- `keyboard_gamepad` — interactive (terminal raw mode). Letters/arrows map to
  gamepad buttons; numeric macros (`1`/`2`/`3`/`0`) emit modifier-then-trigger
  combos that the FSM is listening for.
- `pulse_gamepad <hex_bitmask> <seconds> [iface]` — one-shot publisher for
  scripts. The bitmask follows the standard Unitree `WirelessController_.keys`
  layout.

Bit layout used by both tools (same as
`unitree_sdk2/example/wireless_controller/advanced_gamepad.hpp`):

```
bit 0  R1     bit 4  R2     bit 8  A      bit 12 up
bit 1  L1     bit 5  L2     bit 9  B      bit 13 right
bit 2  start  bit 6  F1     bit 10 X      bit 14 down
bit 3  select bit 7  F2     bit 11 Y      bit 15 left
```

## Why the smoothing fix matters

`unitree::common::UnitreeJoystick::Axis` low-pass filters input with
`smooth=0.03`. That makes sense for noisy analog triggers on a real USB
gamepad, but with a digital DDS source the L2/R2 axis needs ~23 frames to
cross the 0.5 "pressed" threshold. By the time L2 is "pressed", the rising
edge of the paired trigger button (e.g. `up.on_pressed`) has already passed,
so the FSM transition `LT + up.on_pressed` never matches.

Fixes (both required):

- `DDSJoystick` constructor sets `LT.smooth = RT.smooth = 1.0f` (instant on
  the sim side).
- `pulse_gamepad` and `keyboard_gamepad` pre-roll the modifier bits
  (L1/R1/L2/R2) for ~250–300 ms *before* adding the trigger button bits, so
  the controller-side smoothing also gets time to converge.

## Building everything from scratch

```bash
# 1. Simulator (after the simulate/mujoco symlink and CMAKE_PREFIX_PATH are set)
cd /home/ctrob/work/unitree_mujoco/simulate
mkdir -p build && cd build && cmake .. && make -j$(nproc) unitree_mujoco

# 2. unitree_sdk2 examples (optional — useful for sniffers like advanced_gamepad)
cd /home/ctrob/work/unitree_sdk2
mkdir -p build && cd build && cmake .. && make -j$(nproc)

# 3. G1 RL deploy controller
cd /home/ctrob/work/unitree_rl_mjlab/deploy/robots/g1
mkdir -p build && cd build
CXXFLAGS="-I/opt/unitree_robotics/include" \
LDFLAGS="-L/opt/unitree_robotics/lib -Wl,-rpath,/opt/unitree_robotics/lib" \
  cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# 4. Keyboard gamepad tools
cd /home/ctrob/work/unitree_mujoco/tools/keyboard_gamepad
mkdir -p build && cd build && cmake .. && make
```

One-time host setup:

- `sudo ln -sfn /opt/unitree_robotics/include/ddscxx /usr/local/include/ddscxx`
  (the g1 deploy CMakeLists hardcodes `/usr/local/include/ddscxx`).

## Xpra remote display

The default X11 forwarding through VcXsrv/Xming on Windows only advertises
GLX 1.2, but MuJoCo's GLFW needs GLX ≥ 1.3, so `glfwCreateWindow` fails with
`No GLXFBConfigs returned`. Xpra (server on Linux, client on Windows)
forwards through a virtual `Xvfb` that advertises GLX 1.4.

```bash
# Server side (Linux):
xpra start :100 \
  --bind-tcp=0.0.0.0:14500 --tcp-auth=allow --html=off --daemon=yes \
  --opengl=yes --notifications=no --webcam=no --pulseaudio=no --mdns=no \
  --systemd-run=no --printing=no --system-tray=no --dbus-control=no \
  --start-new-commands=no --bell=no
```

Run the sim with `DISPLAY=:100`. On Windows, install the Xpra client from
<https://xpra.org/dists/windows/> and connect to `tcp://<host>:14500/` —
addresses available on this machine:

- `100.121.227.88` (Tailscale)
- `ctrob-workstation.tailb42313.ts.net` (MagicDNS)
- `10.1.82.179` (LAN)

`--printing=no --system-tray=no` are not optional: Xpra 3.1.5 segfaults when
processing the Windows "AnyDesk Printer" system-tray icon. If the server
ever crashes:

```bash
pkill -9 -f 'Xvfb-for-Xpra-:100'
pkill -9 -f 'xpra.*:100'
rm -f /tmp/.X100-lock /tmp/.X11-unix/X100 \
      /home/ctrob/.xpra/ctrob-workstation-100 \
      /run/user/1000/xpra/ctrob-workstation-100 \
      /run/user/1000/xpra/:100.log
# then restart with the same xpra start command above
```

## What works vs. what doesn't

- ✅ `g1_ctrl` low-level (joint torque PD) loop: `rt/lowcmd` ↔ `rt/lowstate`.
- ✅ FSM transitions driven by `wireless_remote` bytes embedded in `rt/lowstate`.
- ✅ RL velocity policy (`config/policy/velocity/v0`) — robot stands and walks.
- ✅ Mimic policy (`config/policy/mimic/dance1_subject2`) — choreographed dance.
- ❌ `g1_loco_client_example` and other high-level DDS-RPC clients (LocoClient,
  AGV, AudioClient, etc.) — these depend on services that run on the real G1's
  onboard computer and are **not** implemented by `unitree_mujoco`. Use the
  `g1_ctrl` FSM (Velocity state) instead — it's the RL-policy equivalent and is
  what would actually run on the robot anyway.

## FSM cheat sheet

| Macro | Bits | Trigger DSL (`config/config.yaml`) | Effect |
|---|---|---|---|
| `1` | `0x1020` | `LT + up.on_pressed` | Passive → FixStand |
| `2` | `0x0110` | `RT + A.on_pressed` | FixStand → Velocity (walking) |
| `3` | `0x0101` | `RB + A.on_pressed` | Velocity → Mimic_Dance1 |
| `0` | `0x0220` | `LT + B.on_pressed` | any RL state → Passive |

In Velocity state: left stick `lx`/`ly` is forward/strafe; right stick `rx` is
yaw rate. With `keyboard_gamepad` press `i k j l` (left stick) and `u o`
(yaw).
