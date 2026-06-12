# Autopilot Lite for C++20

A light, fast multirotor autopilot for offboard control of PX4 / ArduPilot
drones — a mid-2020s replacement for
[mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers).

## Structure

| Package     | Build       | Depends on      | Contents                                         |
| ----------- | ----------- | --------------- | ------------------------------------------------ |
| `apl20`     | plain CMake | Eigen           | Pure-math control kernels. Reusable without ROS. |
| `apl20_ros` | ament_cmake | `apl20`, rclcpp | ROS 2 nodes wrapping the kernels.                |

`apl20_ros` finds `apl20` with `find_package(apl20)` and links the imported
target `apl::rate_control`.

## Controllers

The cascade is **attitude → rate → actuator**: `AttitudeController` turns an
attitude error into a body-rate setpoint, which `RateController` turns into a
normalized torque. Both are pure-math, stateless, and scalar-generic
(`template <std::floating_point Scalar>`).

| Controller                                   | Target                                | Role                          |
| -------------------------------------------- | ------------------------------------- | ----------------------------- |
| `apl::AttitudeController`                    | `apl::attitude_control` (header-only) | attitude → body-rate setpoint |
| `apl::Pid<Scalar,N>` / `apl::RateController` | `apl::rate_control`                   | body-rate → torque            |

### Attitude controller

`apl::AttitudeController` is a tilt-prioritized quaternion controller. The error
is the **SO(3) log** (true angle·axis, linear in the angle — not the
half-angle-sine vector), with the tilt (thrust axis) and yaw read independently
so a large heading error never compromises thrust-axis tracking. Yaw is
deprioritized purely by a lower `kp.z`. Holds only its `Cfg`;
`update(q, qd, yaw_rate_ff)` returns the body-rate setpoint.

### Rate controller

A body-rate PID combining the best of PX4, ArduPilot, and Betaflight into one
opinionated implementation, split into a pure math kernel and a drone driver.

- **`apl::Pid<N>`** — a stateless, Eigen-vectorized N-axis PID kernel. Holds
  only a `PidCfg`; every method is `const`. The evolving `PidState` (integrator,
  derivative history) is owned by the caller and threaded through `update()`,
  which returns the decomposed `PidTerms` (P/I/D/FF). `dt` is injected, never
  read from a clock. D acts on the (low-pass-filtered) measurement; anti-windup
  stacks a soft authority taper, conditional integration against saturation, and
  a hard clamp.
- **`apl::RateController`** — the driver around `Pid<3>`: throttle PID
  attenuation, output normalization/clamping, saturation feedback, and `dt`
  guarding.

```cpp
apl::RateControllerCfg cfg;
cfg.pid.kp = Eigen::Array3d(0.15, 0.15, 0.20);
cfg.pid.ki = Eigen::Array3d(0.20, 0.20, 0.10);
cfg.pid.i_max.setConstant(0.3);
cfg.pid.d_lpf_hz.setConstant(40.0);
apl::RateController rc(cfg);

rc.reset(rate_meas);                                   // on arm
Eigen::Vector3d torque = rc.update(rate_sp, rate_meas, throttle, dt);
```

See `CHANGELOG.md` entry 1 for the design rationale.

## Build & test

```bash
# whole workspace
colcon build --packages-up-to apl20_ros
colcon test --packages-select apl20

# kernel alone, no ROS
cmake -S apl20 -B build/apl20 && cmake --build build/apl20 && ctest --test-dir build/apl20
```

### `apl20_ros` rate controller node

`autopilot_node` runs `RateControllerNode`. Provisional interface (standard
messages — open to swapping for mavros / px4_msgs):

| Topic               | Type                           | Direction                       |
| ------------------- | ------------------------------ | ------------------------------- |
| `~/rate_setpoint`   | `geometry_msgs/Vector3Stamped` | in (body rates [rad/s])         |
| `~/imu`             | `sensor_msgs/Imu`              | in (measured rate + loop clock) |
| `~/throttle`        | `std_msgs/Float64`             | in (normalized [0,1], for TPA)  |
| `~/torque_setpoint` | `geometry_msgs/Vector3Stamped` | out (normalized torque)         |

Gains are node parameters (`rate.kp`, `rate.ki`, `rate.kd`, `rate.kff`,
`rate.i_max`, `rate.d_lpf_hz`, `tpa.breakpoint`, `tpa.rate`, `output_limit`).

### PX4-SITL hover test

`hover_node` (`OffboardCascadeNode`) runs the full attitude→rate cascade against
PX4 SITL over uXRCE-DDS, commanding **torque + thrust** directly (PX4 does only
control allocation, so both our controllers are exercised). It reads
`VehicleOdometry` (attitude + body rates) and `VehicleStatus`, publishes
`VehicleTorqueSetpoint` / `VehicleThrustSetpoint` / `OffboardControlMode`, and
drives the offboard + arm handshake.

```bash
# 1. PX4 SITL (builds + runs Gazebo)
cd ~/src/PX4-Autopilot && make px4_sitl gz_x500
# 2. XRCE bridge + controller
ros2 launch apl20_ros hover.launch.py
# 3. auto-test: arms, goes offboard, reaches altitude -> exit 0
ros2 run apl20_ros hover_check.py
```

Key params: `hover_thrust` (default 0.70), `att.kp`, `att.yaw_weight`,
`att.rate_limit`, and the `rate.*` gains.
