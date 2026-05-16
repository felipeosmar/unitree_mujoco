# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`unitree_mujoco` exposes a MuJoCo simulation to Unitree controller code over DDS. There are two parallel implementations — `simulate/` (C++, recommended) and `simulate_python/` (Python) — that share the same on-the-wire DDS contract, so a controller written against `unitree_sdk2` / `unitree_sdk2_python` / `unitree_ros2` can be pointed at either simulator unchanged. Robot MJCF assets live in `unitree_robots/<robot>/`; examples that drive a simulator (or a real robot) are in `example/`.

## Build & run

### C++ simulator (`simulate/`)

Requires a one-time MuJoCo symlink before the first build:

```bash
cd simulate && ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

`CMakeLists.txt` hard-codes `/opt/unitree_robotics/lib/cmake` as a `CMAKE_PREFIX_PATH`, so `unitree_sdk2` must be installed there (`cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics`). Apt deps: `libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev`.

Build, then run from the `build/` dir (the binary uses its own location to resolve `config.yaml` and the scene path):

```bash
cd simulate && mkdir build && cd build && cmake .. && make -j4
./unitree_mujoco                                  # uses simulate/config.yaml
./unitree_mujoco -r go2 -s scene_terrain.xml      # override robot/scene
./unitree_mujoco -i 1 -n lo                       # override domain_id / interface
```

CLI flags (`simulate/src/param.h`) override the YAML: `-r/--robot`, `-s/--scene`, `-i/--domain_id`, `-n/--network`. A second `jstest` binary is built for debugging gamepad axis/button IDs.

### Python simulator (`simulate_python/`)

No build step. Edit `simulate_python/config.py` (note: `ROBOT` is read at import time and selects the IDL — see Architecture). Then:

```bash
cd simulate_python && python3 unitree_mujoco.py
```

Deps: `unitree_sdk2_python` (installed via `pip3 install -e .` from its repo), `pip3 install mujoco pygame`.

### Test / example controllers

The "test" controller stands the robot up and commands 1 Nm at every motor. There is no test framework — these are runtime smoke tests against a running simulator.

```bash
# C++ example
cd example/cpp && mkdir build && cd build && cmake .. && make -j4
./stand_go2              # talks to sim (domain 1, interface lo)
./stand_go2 enp3s0       # talks to real robot (domain 0, given iface)

# Python examples
python3 example/python/stand_go2.py [iface]
python3 simulate_python/test/test_unitree_sdk2.py

# ROS2 example (requires unitree_ros2 sourced)
cd example/ros2 && colcon build
./install/stand_go2/bin/stand_go2
```

The sim-vs-real switch in every example is the same two-line pattern: no-arg → `Init(1, "lo")`, arg → `Init(0, argv[1])`. Match `domain_id` in `config.yaml` / `config.py` to whatever the controller uses.

### Terrain generation

`terrain_tool/terrain_generator.py` programmatically rewrites a scene XML to add boxes/stairs/heightfields. Edit the constants at the top, then `python3 terrain_generator.py` writes to the path in `OUTPUT_SCENE_PATH` (typically `unitree_robots/<robot>/scene_terrain.xml`).

## Architecture

### Threads and data flow (C++)

`simulate/src/main.cc` starts three threads sharing the global `mjModel* m` / `mjData* d`:

1. **Main / UI** — `mj::Simulate::RenderLoop()` (from upstream MuJoCo `simulate/`, vendored under `simulate/mujoco/simulate/` via the symlink). A custom `user_key_cb` handles elastic-band keys (`7`/`8`/`9`) and `Backspace` for `mj_resetData`.
2. **Physics** — `PhysicsThread` / `PhysicsLoop` steps `mj_step` with the upstream sync/refresh logic, plus injection of the elastic band force on `xfrc_applied` at `band_attached_link` when enabled.
3. **Bridge** — `UnitreeSdk2BridgeThread` waits for `d` to be non-null, then `ChannelFactory::Init(domain_id, interface)` and instantiates one of two bridges based on actuator count.

The Python simulator collapses this to two threads (sim + viewer) in `unitree_mujoco.py`, with a `threading.Lock` (`locker`) guarding `mj_data`. The bridge runs on its own DDS `RecurrentThread`s spawned inside `UnitreeSdk2Bridge.__init__`.

### Robot family selection — the key fork

Two distinct DDS IDL families are used depending on the robot. The simulator **auto-detects** the family at runtime from the loaded model:

- **`unitree_go` IDL** — 20-motor message layout. Used for `go2`, `go2w`, `b2`, `b2w`, `h1`, `a2`.
- **`unitree_hg` IDL** — 35-motor "humanoid/general" layout. Used for `g1`, `h1_2`.

In C++, `main.cc` picks `G1Bridge` vs `Go2Bridge` from `m->nu > NUM_MOTOR_IDL_GO` (20). `G1Bridge` (`simulate/src/unitree_sdk2_bridge.h`) additionally:
- Sets `mode_machine` to 4 for `g1_23dof` scenes, 5 otherwise.
- Publishes `rt/lf/bmsstate` with `soc=100` (fake battery).
- Publishes `rt/secondary_imu` from the `secondary_imu_*` sensors when present in the MJCF.

In Python, `simulate_python/unitree_sdk2py_bridge.py` does **not** auto-detect — it branches on `config.ROBOT == "g1"` at import time to pick the IDL. Changing robots requires editing `config.py`; the Python bridge has fewer features (no `bmsstate`, no `secondary_imu`, no `mode_machine`).

### Bridge ↔ MuJoCo contract

Both bridges read MuJoCo state from named sensors (not joint indices). `_check_sensor()` resolves these names once and caches `sensor_adr`; missing sensors are skipped silently. The expected names are:

| Sensor name              | Published on                          |
| ------------------------ | ------------------------------------- |
| motor pos/vel/torque     | `rt/lowstate` (3 sensors per actuator, in order) |
| `imu_quat/gyro/acc`      | `rt/lowstate.imu_state`               |
| `frame_pos/vel`          | `rt/sportmodestate`                   |
| `secondary_imu_quat/gyro/acc` | `rt/secondary_imu` (G1 only)     |

Each robot's `<robot>.xml` (e.g. `unitree_robots/go2/go2.xml`) must declare actuators and sensors in this exact order — the first `MOTOR_SENSOR_NUM * nu = 3*nu` sensors are read positionally as motor pos / vel / torque, then named sensors follow.

`LowCmd` is consumed in `RobotBridge::run()` as a stiffness controller:
```
ctrl[i] = tau + kp*(q - sensordata[i]) + kd*(dq - sensordata[i + num_motor])
```
i.e. the simulator does the PD math; the actuator XML should expose direct-torque actuators.

### Scene resolution

The C++ simulator resolves `robot_scene` relative paths via:
`<repo>/unitree_robots/<config.robot>/<config.robot_scene>`
(see `main.cc:677`). The Python simulator builds the same path manually in `config.py`. Both expect `scene.xml` (or `scene_terrain.xml`, `scene_23dof.xml`, etc.) inside `unitree_robots/<robot>/`.

### Elastic band (humanoid lift)

When `enable_elastic_band=1`, a virtual spring pulls the body containing `torso_link` (fallback `base_link`) toward a point above the origin. Used to suspend H1/G1 during init. Keys: `9` toggle, `7`/`UP` shorten, `8`/`DOWN` lengthen, `Backspace` resets the sim.

## Things to know

- The `main.cc` `#define private public` before including `glfw_adapter.h` is a deliberate hack to expose `platform_ui->window_` so a key callback can be installed. Don't "clean it up" without rerouting the callback through MuJoCo's UI layer.
- `simulate/mujoco/` is a symlink to a MuJoCo release, not source we own. CMake pulls `glfw_*.cc`, `platform_*.cc`, and `simulate.cc` straight out of it (`simulate/CMakeLists.txt:24`).
- Joystick axis/button IDs are hardcoded per layout in both bridges (`simulate/src/unitree_sdk2_bridge.h` and `simulate_python/unitree_sdk2py_bridge.py`). For unknown gamepads run `jstest` (or `jstest-gtk`) and edit the maps. `joystick_bits=16` vs `8` rescales axis values; some pads report 8-bit.
- `domain_id` defaults to `1` here so a controller running with default `0` will **not** see the sim. Either bump the controller's domain or set `domain_id: 0` (then beware of conflicting with a real robot on the LAN).
