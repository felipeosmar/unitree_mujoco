# G1 on tilting platform — setup & internals

Phase 2 of the remote G1 sim: the robot stands on a dual-axis tilting plate
whose pitch and roll are servo-controlled in **degrees**, with configurable
angular-velocity limit. Two operating modes: `remote` (DDS-driven set-points)
and `boat` (server-side sine-wave generator for water-like motion).

The G1 is **not parented** to the platform — it sits on top under friction, so
the on-robot controller (`g1_ctrl` from `unitree_rl_mjlab`) has to keep its
balance just like on the real robot.

---

## TL;DR — one-command start

```bash
/home/ctrob/work/unitree_mujoco/scripts/start_g1_platform.sh
```

Wait ~12 s. When the script prints `G1 is now active on the level platform`,
open the Xpra client on Windows — you'll see the G1 standing in the middle of
a flat blue 2 × 2 m plate, in **Velocity** mode (RL walking policy live).

Then in any other terminal:

```bash
# Drive the platform manually:
~/work/unitree_mujoco/tools/platform/build/keyboard_platform
#   w/s/a/d = pitch/roll ±1°,  Shift = ±5°,  [/] = rate,  r = reset, q = quit

# Or single tilts from a script:
~/work/unitree_mujoco/tools/platform/build/pulse_platform 10 0 30 2 lo
#   pitch=10°, roll=0°, max_rate=30 °/s, duration=2 s, iface=lo
```

`Ctrl+C` in the launch terminal stops everything and restores `config.yaml`.

---

## Manual setup (without the script)

Edit `simulate/config.yaml`:

```yaml
robot: "g1"
robot_scene: "scene_platform.xml"
domain_id: 0
interface: "lo"
use_joystick: 1
joystick_type: "dds"
enable_elastic_band: 0      # the platform mode runs without the band
platform_mode: "remote"     # "off" | "remote" | "boat"
platform_kp: 3000.0         # stiffer than the default 800 so the plate stays
platform_kd: 200.0          #   level even with an unbalanced robot on top
```

Then run the usual three processes (each in its own SSH terminal):

```bash
# Terminal 1 — sim
cd ~/work/unitree_mujoco/simulate/build && DISPLAY=:100 ./unitree_mujoco

# Terminal 2 — G1 controller (FSM + RL policies)
cd ~/work/unitree_rl_mjlab/deploy/robots/g1/build && ./g1_ctrl --network=lo

# Terminal 3 — drive the FSM (Passive -> FixStand -> Velocity)
~/work/unitree_mujoco/tools/keyboard_gamepad/build/keyboard_gamepad
#   press 1 (L2+up: FixStand), then 2 (R2+A: Velocity)
```

---

## Operating modes

`platform_mode` in `config.yaml` picks the behaviour:

| Mode | Behaviour |
|---|---|
| `off`    | Hinges are free (with damping). Plate reacts only to gravity / contacts. |
| `remote` | PD targets come from `rt/platform_cmd` (see DDS contract below). Rate-limited per command. |
| `boat`   | Server-side sine-wave generator: <br>`pitch = A_p · sin(2π t / T_p)` <br>`roll  = A_r · sin(2π t / T_r + φ)` <br>All parameters live in `config.yaml` under `platform_boat:`. |

The actual platform pose is always published on `rt/platform_state` at ~50 Hz,
regardless of mode.

### Boat-mode tuning

```yaml
platform_boat:
  pitch_amp_deg: 5.0          # peak pitch in degrees
  pitch_period_s: 4.0         # period in seconds
  roll_amp_deg: 7.0
  roll_period_s: 3.0
  phase_offset_rad: 1.5708    # pi/2 — pitch/roll 90° out of phase = circular bobbing
```

To switch to boat mode while the sim is running:
1. `Ctrl+C` the launch script (or kill the sim manually)
2. Edit `config.yaml`: `platform_mode: "boat"`
3. Restart

---

## DDS contract

Both topics reuse the standard `unitree_go::msg::dds_::WirelessController_` IDL
as a generic 4-float + uint16 carrier. No custom IDL was generated — that keeps
the pipeline drop-in compatible with the existing tools.

### `rt/platform_cmd` — set-points (sim subscribes when mode = `remote`)

| Field | Meaning |
|---|---|
| `lx`   | target pitch (deg, positive = nose-up around X) |
| `ly`   | target roll  (deg, positive = right-side-up around Y) |
| `rx`   | max angular rate (deg/s). `0` keeps the default from `platform_default_max_rate_deg_s`. |
| `ry`   | unused |
| `keys` | unused |

### `rt/platform_state` — current pose (sim publishes)

| Field | Meaning |
|---|---|
| `lx`   | actual pitch (deg) |
| `ly`   | actual roll  (deg) |
| `rx`   | actual pitch rate (deg/s) |
| `ry`   | actual roll  rate (deg/s) |
| `keys` | always 0 |

---

## Tools (`tools/platform/`)

`pulse_platform`
:   One-shot scriptable publisher.
    Usage: `pulse_platform <pitch_deg> <roll_deg> [rate_deg_s=0] [dur_s=0.5] [iface=lo]`

`keyboard_platform`
:   Interactive driver in raw TTY mode. Subscribes to `rt/platform_state` and
    prints actual vs target every 200 ms. Keys:
    - `w`/`s` → pitch ±1° (`W`/`S` = ±5°)
    - `a`/`d` → roll ±1° (`A`/`D` = ±5°)
    - `[`/`]` → adjust max rate ±10 °/s
    - `space` → zero target
    - `r` → reset rate to 30 °/s
    - `q` or `Esc` → quit

---

## How the sim implements the platform

Three pieces, all opt-in (the platform is invisible to any scene that doesn't
declare `platform_pitch` / `platform_roll` joints).

### 1) `unitree_robots/g1/scene_platform.xml`

Wraps the standard `g1_29dof.xml` model. Adds:

- A pitch hinge body and a nested roll hinge body, both with explicit inertials
  (MuJoCo requires inertials on bodies with moving joints).
- The plate geom — `box size="1.0 1.0 0.05"` (10 cm thick). It's intentionally
  thick because the G1's foot visual mesh extends ~5 cm below its actual
  collision points (4 small spheres at z=-0.03 on the ankle_roll_link); a thin
  plate would make the mesh look like it's clipping out the bottom.
- Four cosmetic rim geoms with `contype="0" conaffinity="0"` (no collision).
- A catch-all ground plane at `z = -2` so a fall off the plate doesn't drop
  the robot into the void.
- A `<keyframe name="fixstand">` with `qpos` pre-set to the FixStand joint
  configuration. The G1 spawns in stand pose instead of zero-pose.

### 2) `simulate/src/platform_controller.h`

A small C++ class instantiated unconditionally in `UnitreeSdk2BridgeThread`. It
auto-detects the platform by looking up the joint names — if missing, it's a
no-op. Otherwise it:

- Subscribes to `rt/platform_cmd` (when mode = `remote`).
- Each physics step (under the sim mutex) it:
  - Pulls the current target — from boat math or the latest DDS msg.
  - Rate-limits the setpoint by `max_rate_deg_s · dt`.
  - Computes `τ = kp · (target − q) − kd · q̇` and writes it to
    `qfrc_applied[dof_adr]` for each platform hinge.
- Publishes pose on `rt/platform_state` every ~20 ms.

> **Why `qfrc_applied` and not MuJoCo actuators?** Adding actuators to the XML
> bumps `m->nu` past 29 (the G1's motor count), which breaks the bridge code
> that reads the first `3 · nu` sensors as motor pos/vel/torque. Writing the
> torque directly to `qfrc_applied` keeps `m->nu = 29` and the motor mapping
> untouched.

### 3) Bootstrap stand pose (`G1Bridge` constructor)

The G1's on-robot controller (`g1_ctrl`) takes ~5–8 s to boot — DDS discovery,
ONNX policy loading, motion-file parsing. During that window the sim is
running but the bridge has no `lowcmd` to read, so it applies zero torque to
every motor and the robot collapses limp. It would land sprawled on (or off)
the plate before the controller could engage FixStand.

The fix: the `G1Bridge` constructor pre-fills the lowcmd subscriber's `msg_`
with the FixStand joint targets plus a soft holding PD (`kp=80`, `kd=5`). The
bridge applies those torques until the controller publishes its first real
`lowcmd`, which then overwrites `msg_` transparently. Net effect: the robot is
actively standing from `t=0`.

```cpp
// excerpt from G1Bridge ctor
std::lock_guard<std::mutex> lock(this->lowcmd->mutex_);
for (int i = 0; i < std::min(num_motor_, 29); ++i) {
    auto& mc = this->lowcmd->msg_.motor_cmd()[i];
    mc.q() = kStandQ[i];          // FixStand joint angles
    mc.kp() = 80.0f;              // soft hold so the robot doesn't fight
    mc.kd() = 5.0f;               //   the real controller when it arrives
    mc.mode() = 1;
}
```

---

## Convenience script — what it does, in order

`scripts/start_g1_platform.sh`:

1. Backs up `simulate/config.yaml` to `config.yaml.bak.start_g1_platform`.
2. Rewrites the live config to:
   - `robot_scene: "scene_platform.xml"`
   - `enable_elastic_band: 0`
   - `platform_mode: "remote"`
   - `platform_kp: 3000.0`, `platform_kd: 200.0`
   (Uses `[^"]*` in the `sed` patterns so inline comments aren't swallowed — a
   greedy `.*` previously produced an invalid YAML that aborted the sim.)
3. Launches `unitree_mujoco` and `g1_ctrl` in parallel (`g1_ctrl` blocks on
   `wait_for_connection` until the sim publishes its first lowstate).
4. Polls `g1_ctrl`'s log for `FSM: Start Passive`.
5. Pre-seeds a zero target on `rt/platform_cmd` so the platform's "remote"
   target is unambiguously level.
6. Sends `pulse_gamepad 0x1020` (L2+up) → `Passive → FixStand` transition.
   Waits for the log line confirming it. Retries once if it doesn't fire.
7. Sends `pulse_gamepad 0x0110` (R2+A) → `FixStand → Velocity` transition.
8. Reports the PIDs, log paths, and useful follow-up commands. Blocks waiting
   on either child to exit; `Ctrl+C` triggers cleanup (kill children, restore
   config).

Override-able environment variables:

| Var | Default |
|---|---|
| `DISPLAY`   | `:100` |
| `IFACE`     | `lo` |
| `RL_DEPLOY` | `~/work/unitree_rl_mjlab/deploy/robots/g1` |
| `SIM_LOG`   | `/tmp/sim.log` |
| `CTRL_LOG`  | `/tmp/g1ctrl.log` |

---

## Known caveats

- **The `WirelessController_` IDL is reused as a generic carrier** for the
  platform topics. Functional but semantically a bit odd — if it ever becomes
  confusing, generating a dedicated `PlatformCmd_/PlatformState_` IDL is ~1 h
  of work.
- **High-stiffness platform PD (kp=3000) ≠ rigid table.** A G1 in Velocity
  mode pumping hard on the plate can still tilt it a degree or two
  transiently. If you want a "rigid floor that tilts" feel, bump kp toward
  10 000 — but the larger the kp, the more your physics timestep has to drop
  to stay stable.
- **`scene_platform.xml`'s keyframe assumes 29 G1 motors + 2 platform joints**
  (qpos length 38). If you swap to `g1_23dof`, the keyframe count is wrong and
  the loader will error. Either skip the keyframe load for that variant or
  write a separate `scene_platform_23dof.xml`.
- **The boat mode runs entirely server-side.** The `keyboard_platform` tool
  has no effect in boat mode — switch to `remote` first.
