# Changelog

Long-form narrative for `apl20`. Newest entries on top. Commits reference an
entry by number, e.g. `See CHANGELOG entry 1.`

## 10. Safe direct-actuator offboard: operator-gated arming + honest thrust setpoint

Two fixes to the offboard `actuator_motors` path so it respects the vehicle's
actual armed/offboard state rather than forcing and faking it.

**Arming becomes the operator's decision.** The node used to stream a non-zero
(hover) collective every cycle and, via `maybeRequestOffboardArm`, self-command
`DO_SET_MODE→OFFBOARD` + `ARM` ~1 s after the first odometry — so it armed itself
and the props jumped straight to hover. Now the motors are driven only when PX4
reports the vehicle *actually* armed AND in offboard (`isEngaged`); until then
they are commanded explicitly stopped (all-NaN `ActuatorMotors`) while the
`OffboardControlMode` heartbeat keeps offboard *selectable*. On the
disengaged→engaged edge the shapers and loops re-seed at the current pose
(`seedControllers`), so control starts from where the vehicle is and the takeoff
guard climbs from the ground — no stale-hover jump. The old self-arming handshake
is retained behind `auto_engage` (default false; `autopilot.launch.py` exposes
it) for headless SITL.

**An honest thrust setpoint keeps the land detector sane.** In direct-actuator
offboard PX4 runs no allocation, so `vehicle_thrust_setpoint` is never populated
and sits at zero. PX4's `MulticopterLandDetector` reads throttle = `-xyz[2]` from
it, so a zero setpoint reads as zero throttle: on a *settled* hover the
low-throttle and minimum-thrust conditions latch and it walks ground_contact →
maybe_landed → landed, auto-disarming mid-air. The node now mirrors the
collective/torque it allocates onto `vehicle_thrust_setpoint` (`xyz.z =
-collective`, so the detector's `-xyz[2]` is the true throttle) and
`vehicle_torque_setpoint`, published only while engaged (from `publishMotors`, so
PX4 owns those topics in every other mode). apl20 keeps its own control allocator
(`direct_actuator`); the setpoints exist purely to feed the detector and logging.

## 9. Waypoint missions + tracking-performance recording

The single target pose became a **waypoint mission**. `apl::mission` (header-only)
adds a `Waypoint` (NED position + heading) and two survey-pattern generators that
emit absolute-NED waypoint lists: `SquarePattern` (closed 4-corner loop) and
`LawnmowerPattern` (boustrophedon lanes covering a rectangle, last lane clamped to
the far edge for exact coverage). `OffboardCascadeNode` builds one on capture from
the `pattern` param (hover | square | lawnmower) centred on the target pose,
phased-takes-off at the start heading, then navigates the waypoints in turn —
advancing once settled within `accept_radius` and below `settle_speed`. Each leg
is still a smooth jerk-limited `PositionReference` trajectory, so the corners are
rounded rather than stopped-and-jerked. `autopilot.launch.py` exposes the pattern
and its dimensions.

The **tracking-performance framework**: `apl20_ros::TrackingRecorder` logs, every
control cycle, the shaped reference the controller is tracking vs the measured
pose (plus position/heading error) to a CSV — `track_log` param sets the path, ''
disables. Recording the *shaped* reference (not the raw waypoint) is the point: it
measures how well the airframe follows the feasible trajectory, not how far the
distant waypoint is. `scripts/track_analyze.py` reads the CSV and reports overall
and per-waypoint RMS / max / mean error, the raw material for tuning and
regression. The CSV schema and the analyzer are validated against each other.

## 8. Position reference shaper: smooth pose setpoints (no jump)

`apl::PositionReference` (header-only `apl::position_reference`) — the
trajectory-generation layer above the `PositionController`, the position analog of
`AttitudeReference`. The node's fixed hover became a user-specified target *pose*:
the `target.{x,y,altitude,yaw}` params set an absolute point in the local NED
frame (origin at EKF home), the shaper -- seeded at the current pose -- ramps an
internal "achievable" (pos, vel, accel) state toward it through a
jerk-limited kinematic S-curve, and the controller only ever tracks that feasible
reference. Hover is the trivial case — target (0, 0, altitude, 0) directly
overhead, the state ramps up from rest and settles.

This fixes the step-setpoint jump: a naive Lee/Sreenath/Mellinger tracker fed a
step (current -> far waypoint, or `[0;3] -> [0,0,z]`) produces a huge instant
velocity command and lurches. Here the *reference* moves smoothly under per-axis
velocity / acceleration / jerk limits, so it never demands a rate the airframe
can't deliver — PX4/ArduPilot's smooth POSCTL takeoff and translation. Ported
from ArduPilot's `shape_pos_vel_accel`: sqrt controller pos->vel, sqrt controller
vel->accel, jerk-slew the accel, integrate accel->vel->pos — built from the same
`SqrtController`/`SlewRate` kernels `AngularInputShaper` uses, now factored into
their own `apl::math` target so the attitude and position shapers share them.

Kept a separate stateful component (like the attitude shaper): the controller
stays a pure feedback law, the shaper owns the (pos, vel, accel, yaw) target. The
unit suite checks the property that matters — a step climb's first acceleration is
jerk-limited (ramps from zero, no jump), the trajectory respects every limit
throughout, it converges and settles, and the overhead/hover case stays purely
vertical. `autopilot.launch.py` becomes the single, continuously-evolved entry
point (consolidating the old `hover`/`pose` launches), with hover as its zero-arg
default — named for the system rather than any one behaviour.

A **phased takeoff** makes the absolute target safe from the ground. The vehicle
can't yaw or translate while sitting on its feet, so commanding an absolute pose
that differs from where it spawned (in SITL the x500 faces ~96°, not north) only
winds the yaw torque to saturation — which the airmode allocator then trades
collective away to honour, starving the climb. So until it has climbed
`takeoff_alt` (0.5 m) the node holds the start xy + heading and climbs straight
up; once airborne it switches to the full absolute target and the shaper
transitions smoothly. That is PX4/ArduPilot's "vertical takeoff, then navigate".

Validated in PX4 SITL (gz x500): bounded jerk-limited climb, airborne, yaw to the
absolute heading, then holds ~2.00 m to **std 11 mm**. (The debugging also pinned
the hover thrust in our linear direct mapping at ~0.73 — the apparent thrust
saturation seen first was the yaw torque eating collective, not a thrust shortfall.)

## 7. Direct actuator control: our own control allocation

`apl::ControlAllocator` (header-only `apl::control_allocation`) — the last
link of the stack, mapping the rate controller's body torque + the position
controller's collective thrust to **per-motor** normalized commands. With it the
offboard node commands PX4 at the `direct_actuator` level (`ActuatorMotors`), so
PX4 now runs *none* of its own control — not even the mixer. We own everything
from position down to the motors.

The mix is the Moore-Penrose pseudo-inverse of the rotor effectiveness, built
from geometry alone (`RotorEffectiveness`: `tau_x = -arm_y`, `tau_y = arm_x`,
`tau_z = km`, `collective = 1`, i.e. PX4's `ActuatorEffectivenessRotors` with the
thrust coefficient dropped — it cancels under normalization). Crucially the
columns are normalized *exactly* as PX4's `ControlAllocationPseudoInverse` (roll &
pitch share one RMS scale `sqrt(||col||^2 / (n/2))`, yaw by its peak coefficient,
collective by mean magnitude), so the same `(torque, thrust)` we previously handed
PX4's allocator in torque+thrust mode now yields the same motors — **the
rate-controller tuning carries over unchanged**. The 4x4 inverse runs once at
config time, not in the loop.

Desaturation is airmode-style: when a motor would exceed a rail, shift *all*
motors equally to pull it back. On a symmetric frame each effectiveness torque
row sums to zero, so a uniform shift is torque-neutral — it preserves
roll/pitch/yaw authority and sacrifices only absolute thrust. A deliberate
single-pass simplification of PX4's sequential, per-axis desaturation, per the
project's "no edge-case handling" ethos.

The kernel stays in the apl20 mould: stateless, holds only its `Cfg`, `allocate()`
pure and `const`, scalar- and motor-count-generic. Built and unit-tested against
the PX4 SITL x500 geometry (airframe 4001_gz_x500): a pure collective spreads
evenly, each torque axis produces the right differential with the collective
preserved, torque and thrust are decoupled below saturation, the airmode shift is
exact, and extreme demands clamp into `[0,1]`. `OffboardCascadeNode` now publishes
`ActuatorMotors` (`control[0..3]`, the rest `NaN` = stopped) with
`OffboardControlMode.direct_actuator`.

Validated in PX4 SITL (gz x500): with our `ControlAllocator` driving the motors,
the x500 armed, entered offboard, climbed to the 2 m setpoint and **held it to
mean 1.97 m, std 9 mm (range 1.95–1.98 m) over 15 s** — a rock-solid hover,
confirming the motor order, signs and scaling are right (a wrong sign flips the
craft on arm and never reaches the setpoint). One direct-actuator caveat
surfaced and is worth recording: PX4's multicopter land detector reads the
*thrust setpoint* to decide "landed", which we don't publish in this mode, so it
chatters landed/airborne and auto-disarms — killing a sustained hover. Disabling
auto-disarm-on-land (`COM_DISARM_LAND -1`) lets our own loop hold; the land
detector is the only PX4 subsystem that still wants the old thrust-setpoint path
we left behind.

## 6. Attitude reference shaper + feedforward (ArduPilot input shaping)

`apl::AttitudeReference` — the reference-generation layer above the (stateless)
`AttitudeController`, the attitude analog of a waypoint smoother above the
position controller. It slews an internal "achievable" `attitude_target_` toward
the raw desired attitude under acceleration/velocity limits (so the rate loop is
never commanded a rate the airframe can't reach) and emits the target's angular
velocity as **feedforward**. Ported from ArduPilot's `AC_AttitudeControl` input
shaping: `SqrtController` (the piecewise P/sqrt finite-time, no-overshoot law),
`SlewRate`/`InputShapeAngle`, and the SO(3) `AngleAxisToQuaternion` exp map, all
in `math.hpp`.

Kept as a *separate stateful component*, not folded into the controller: the
controller stays a pure error->rate kernel, and the feedforward is combined with
the feedback in one explicit, testable line — `CombineAttitudeRate` — which
de-prioritizes the feedforward (and holds yaw to the gyro) as the tilt error
grows, ArduPilot's `feedforward_scalar`.

This was prompted by reviewing FSC-Lab's `fsc_autopilot_ros2` APM controller: it
ported the whole architecture faithfully but, in `attitudeControllerRunQuat`,
computed the large-tilt-error feedforward blend into a local and then returned
the *un-blended* feedback — so the de-prioritization was dead code and only the
nominal (< 30 deg) path ever ran. It flew because a hover never trips that
branch. Our `CombineAttitudeRate` returns the blended result, and the test suite
explicitly exercises the large-error path their flights never hit.

## 5. Geometric position controller, robust to mass uncertainty

`apl::PositionController` (header-only `apl::position_control`) — the outer
loop that closes the altitude/position loop the open-loop-thrust hover lacked.
Structure is Lee/Sreenath geometric: position-P -> velocity setpoint; velocity
loop -> acceleration setpoint; acceleration + gravity -> a thrust vector that
splits into a collective magnitude + a desired tilt (attitude), feeding
AttitudeController. NED throughout.

The design question was robustness to mass/hover-thrust uncertainty: a naive
geometric tracker computes thrust as a function of a *known* mass, so any error
becomes steady-state altitude drift (the sibling `autopilot` project's
`GeometricPositionController` is exactly this — PD only, no integral, relying on
a `QuadrotorModel`). Survey of PX4 (`PositionControl` `_vel_int` + optional
hover-thrust Kalman EKF), ArduPilot (velocity/accel I-term + 10 s throttle-hover
learning), and iNav (velocity I-term, fixed hover throttle): **all rely on the
velocity-loop integral term**; none use a UDE/disturbance observer. The integral
absorbs the lumped disturbance (mass error + wind + battery sag); online
hover-thrust estimation is only an optional refinement.

So the velocity loop here **reuses the `apl::Pid<3>` kernel** — its integral,
clamp, and conditional (saturation-aware) anti-windup are exactly what the
velocity loop needs. `hover_thrust` is the operating point the integral corrects
around. Verified in a 40 s vertical sim: configured with the wrong hover thrust
(0.5 vs a true 0.7), the integral converges to 0.7 and holds altitude. The Pid
kernel, built for rate control, dropping cleanly into the velocity loop is a nice
proof of its generality.

Wired into `apl20_ros/hover_node` as the outer loop (`vehicle_local_position_v1`
NED -> position -> attitude + thrust -> rate -> torque) and flown in PX4 SITL
(gz x500): it captures the startup pose, climbs to a 2 m setpoint, and **holds
altitude to ~2.05 m with < 0.5 deg tilt for 75 s+** — no drift, no flyaway. The
velocity integral picks up the x500's real hover thrust on its own, exactly as
the unit sim predicted. This turns the previously open-loop ("can't hold
altitude") hover into a stable closed-loop one.

## 4. Attitude error: SO(3) log, tilt-prioritized

The attitude controller's error vector switched from `2 * vec(qe) = 2 sin(θ/2) *
axis` (PX4/Brescianini) to the **SO(3) log** `θ * axis` — the true rotation angle
about each axis. `2 sin(θ/2)` is linear only for small θ and *saturates* at 2
(θ = π); the log is linear all the way to π, so large attitude errors get a
proportional correction instead of a compressed one (a 172° yaw error now yields
`kp.z · 3.0`, not `kp.z · 1.995`). Imported Ceres's `QuaternionToAngleAxis`
(quaternion log, with the `θ > π` wrap folded into an `atan2`) into
`apl20/math.hpp`.

Tilt and yaw are now read **independently** — the tilt as `log(swing)` (xy) and
the yaw as the true heading angle `2·atan2(qe.z, qe.w)` (z) — so yaw never
couples into the thrust-axis correction. That makes tilt-prioritization
*structural*, which retired PX4's `_yaw_w` knob (and its `1/yaw_weight` gain
compensation): yaw is simply deprioritized by a lower `kp.z`. The
`AttitudeControllerCfg` loses `yaw_weight`.

This is apl20 taking the "best opinionated" path over strict PX4 parity, per the
deviation documented in the sibling `autopilot` project's tilt-prioritized law.
Reference: Brescianini & D'Andrea, "Tilt-Prioritized Quadrocopter Attitude
Control" (2020).

## 3. PX4-SITL offboard hover test (torque + thrust)

`apl20_ros` gains a `hover_node` (`OffboardCascadeNode`) that runs the full
`AttitudeController -> RateController` cascade against PX4 SITL over the
uXRCE-DDS bridge, commanding at the **torque + thrust** level so both of our
controllers are under test (PX4 does only control allocation).

- **One measurement source:** `VehicleOdometry` carries both the attitude
  quaternion (FRD->NED) and the body angular velocity, so it is the loop's
  measurement *and* clock. (`vehicle_angular_velocity` is disabled in PX4's
  `dds_topics.yaml`, so odometry is the practical source.)
- **Outputs:** `VehicleTorqueSetpoint` + `VehicleThrustSetpoint` (thrust is `-z`
  in FRD = up), with `OffboardControlMode.thrust_and_torque = true` streamed
  every cycle, all on best-effort QoS to match the uXRCE client.
- **Handshake:** streams setpoints for ~1 s, then requests OFFBOARD
  (`DO_SET_MODE` 1/6) and arms (`COMPONENT_ARM_DISARM`), retrying until PX4
  reports both — per the `COM_OF_LOSS_T` freshness rule.
- **Hover hold:** level attitude captured at the startup heading + a fixed hover
  thrust (open-loop altitude; the position controller is future work).

Ships with `launch/hover.launch.py` (XRCE agent + node) and a `hover_check.py`
pass/fail auto-test (armed + offboard + reached altitude). Validated end-to-end
in a PX4-free dry run: fed synthetic odometry/status, confirmed correct
`offboard_control_mode`, thrust `(0,0,-0.7)`, zero torque at zero error, and the
`DO_SET_MODE`/arm command stream.

## 2. Brescianini attitude controller

The outer loop of the cascade, ported from PX4's `mc_att_control/AttitudeControl`
into `apl::AttitudeController` (header-only target `apl::attitude_control`).
Maps an attitude error to a body-rate setpoint that feeds the rate controller.

Like the PID kernel it is stateless (holds only its `Cfg`, `update()` is pure and
`const`) and scalar-generic (`template <std::floating_point Scalar>`, so it can
run in `float` to match PX4 bit-for-bit or `double` for headroom). It uses
`Eigen::Quaternion` for the attitude math.

The algorithm is the quaternion controller of Brescianini, Hehn & D'Andrea
(2013): it computes the *reduced* (tilt-only) attitude error from the body-z
axes, then mixes back a `yaw_weight`-scaled share of the yaw error — so a large
heading error never slows down thrust-axis tracking. PX4's `1/yaw_weight` yaw-gain
compensation is reproduced, keeping the small-angle yaw response independent of
the weight. A world-frame yaw-rate feedforward is rotated into the body frame and
added, and the result is clamped per axis.

Dropped PX4 cruft: uORB, the parameter system, manual-stick setpoint generation,
thrust/throttle curves, armed/landed/VTOL state handling — leaving only the
~10-line math core. `setAttitudeSetpoint`'s stored `qd`/`yawspeed` become plain
`update()` arguments.

## 1. PID rate controller: pure-math kernel + drone driver

The first control law: a body-rate PID for offboard multirotor control, split
across the two packages per the project structure.

### Package split

- **`apl20`** (pure CMake, Eigen-only, no ROS) — the reusable math:
  - `apl::Pid<N>` — a stateless, vectorized N-axis PID *kernel*.
  - `apl::RateController` — the drone-facing *driver* around `Pid<3>`.
  - Consumed downstream as the imported target `apl::rate_control`.
- **`apl20_ros`** (ament) — `find_package(apl20)` and a thin `RateControllerNode`
  wrapping the driver in ROS subscriptions/publications.

### The kernel — transcending the Arduino design

The brief was to take the minimal embedded `Arduino-PID-Library` as a starting
point but *transcend its mutable state / input / time design*. That library
bakes the input, output, setpoint pointers, the integrator, and a `millis()`
clock into the object as mutable members. We invert all three:

- **State is explicit.** `Pid` holds only its `PidCfg` and every method is
  `const`. The evolving data (integrator, derivative history) lives in a
  caller-owned `PidState` threaded through `update()`. The kernel is pure.
- **Time is injected.** `dt` is an argument; there is no internal clock.
- **No I/O coupling.** Inputs and outputs are values, not linked pointers.

The kernel is vectorized over `N` axes with Eigen arrays (PX4's `Vector3f`
`emult` lesson) so roll/pitch/yaw are one branch-free expression. `update()`
returns the decomposed `PidTerms` (P/I/D/FF) — ArduPilot's `AP_PIDInfo` idea —
for logging and tuning without reaching inside the controller.

### Synthesis of the four references

- **D-on-measurement** (Betaflight, PX4, and the Arduino `dInput`): the
  derivative acts on the measurement, not the error, so a setpoint step produces
  no derivative kick. It is low-pass filtered per axis (Betaflight / ArduPilot).
- **Feed-forward** `kff * setpoint` (PX4).
- **Anti-windup**, three stacked mechanisms:
  1. soft integral-authority taper as `|error|` grows — PX4's `i_factor`;
  2. conditional integration against a saturated output — PX4's allocator
     feedback / ArduPilot's `limit` flag, generalized to per-axis hi/lo masks;
  3. a hard clamp on the accumulated state (all three stacks).
- **Gain-scheduling hooks** `pd_scale` / `i_scale` (ArduPilot), kept as pure
  math in the kernel; *what* to schedule on is the driver's business.

Per the "single opinionated implementation, no edge-case handling" target, the
deliberately-dropped cruft includes Betaflight's fixed-point gain scales and
scattered flight-mode branches, ArduPilot's target/error pre-filters and slew
limiter, and PX4's module/uORB plumbing.

### The driver — where the drone logic lives

`RateController` owns the `PidState` and adds exactly the concerns the kernel
refuses to know about: Betaflight **throttle PID attenuation** (mapped onto
`pd_scale`), output normalization and clamping to the actuator range, deriving
the saturation signal from *its own* clamp (so no external control allocator is
required), and `dt` glitch-guarding.
