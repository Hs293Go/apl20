#ifndef APL_CONTROL_ALLOCATOR_HPP_
#define APL_CONTROL_ALLOCATOR_HPP_

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>

namespace apl {

// Reduced actuator-effectiveness matrix E (4 x NumMotors): maps per-motor
// normalized thrust to the body wrench [tau_x, tau_y, tau_z, collective] (FRD,
// thrust up = positive collective). `geometry` is one row per motor,
// [arm_x, arm_y, km] -- the rotor's body-frame arm [m] and the dimensionless
// moment ratio km whose sign encodes spin direction (PX4's
// CA_ROTOR{i}_{PX,PY,KM}). Per PX4's ActuatorEffectivenessRotors with axis
// (0,0,-1): moment = ct*(p x axis) - ct*km*axis, giving
//   tau_x = -arm_y,  tau_y = arm_x,  tau_z = km.
// The common thrust coefficient ct is dropped -- it cancels under the
// normalization in NormalizedMix -- and the collective row is +1 (thrust up) so
// a positive collective drives positive motors.
template <std::floating_point Scalar, int NumMotors>
Eigen::Matrix<Scalar, 4, NumMotors> RotorEffectiveness(
    const Eigen::Matrix<Scalar, NumMotors, 3>& geometry) {
  Eigen::Matrix<Scalar, 4, NumMotors> e;
  e.row(0) = -geometry.col(1).transpose();  // tau_x (roll)  = -arm_y
  e.row(1) = geometry.col(0).transpose();   // tau_y (pitch) =  arm_x
  e.row(2) = geometry.col(2).transpose();   // tau_z (yaw)   =  km
  e.row(3).setOnes();                       // collective (thrust up)
  return e;
}

// Moore-Penrose mix B (NumMotors x 4) from a full-row-rank effectiveness E:
// motor = B * [tau_x, tau_y, tau_z, collective], the minimum-norm allocation.
// Columns are normalized exactly as PX4's pseudo-inverse allocator
// (ControlAllocationPseudoInverse), so a unit command uses the same share of
// motor authority PX4 would grant it: roll & pitch share one scale
// sqrt(||col||^2 / (n/2)) (n = motors contributing to that axis), yaw by its
// peak coefficient, collective by mean magnitude (so collective [0,1] passes
// through to motors [0,1]). The scaling is independent of the rotor thrust
// coefficient, so geometry alone determines the mix. (E E^T) is 4x4, inverted
// directly -- this runs once at configuration time, not in the loop.
template <std::floating_point Scalar, int NumMotors>
Eigen::Matrix<Scalar, NumMotors, 4> NormalizedMix(
    const Eigen::Matrix<Scalar, 4, NumMotors>& e) {
  Eigen::Matrix<Scalar, NumMotors, 4> b =
      e.transpose() * (e * e.transpose()).inverse();

  const Scalar eps = Scalar(1e-3);
  const auto axis_scale_rms = [&](int col) {
    const Scalar n =
        static_cast<Scalar>((b.col(col).array().abs() > eps).count());
    return n > Scalar(0) ? std::sqrt(b.col(col).squaredNorm() / (n / Scalar(2)))
                         : Scalar(1);
  };

  // Roll & pitch: one shared scale (the larger), so their relative authority is
  // preserved.
  const Scalar rp_scale = std::max(axis_scale_rms(0), axis_scale_rms(1));
  if (rp_scale > eps) {
    b.col(0) /= rp_scale;
    b.col(1) /= rp_scale;
  }

  // Yaw: by its peak coefficient (yaw is the weakest axis and desaturated
  // first, so PX4 scales it to a unit peak rather than RMS).
  const Scalar yaw_scale = b.col(2).maxCoeff();
  if (yaw_scale > eps) {
    b.col(2) /= yaw_scale;
  }

  // Collective: by mean magnitude.
  const Scalar n_thrust =
      static_cast<Scalar>((b.col(3).array().abs() > eps).count());
  if (n_thrust > Scalar(0)) {
    b.col(3) /= b.col(3).cwiseAbs().sum() / n_thrust;
  }

  // Drop negligible entries so the desaturation sees clean zeros.
  b = (b.array().abs() < eps).select(Scalar(0), b.array()).matrix();
  return b;
}

template <std::floating_point Scalar, int NumMotors>
struct ControlAllocatorCfg {
  // motor = mix * [tau_x, tau_y, tau_z, collective]; build with NormalizedMix.
  Eigen::Matrix<Scalar, NumMotors, 4> mix =
      Eigen::Matrix<Scalar, NumMotors, 4>::Zero();
  Scalar motor_min = Scalar(0);  // idle (motors never stop in flight)
  Scalar motor_max = Scalar(1);  // full thrust
};

// One rotor of a multirotor airframe -- the declarative unit of vehicle
// geometry. `arm_x`/`arm_y` are its body-frame position [m] (forward +, right
// +) and `km` the moment ratio (yaw torque per unit thrust) whose sign encodes
// spin direction (CCW > 0, CW < 0). Mirrors PX4's CA_ROTOR{i}_{PX,PY,KM}.
template <std::floating_point Scalar>
struct Rotor {
  Scalar arm_x;
  Scalar arm_y;
  Scalar km;
};

// Build a ControlAllocatorCfg from a declarative list of rotors, one per motor
// in actuator-output order. Encapsulates the whole geometry -> effectiveness ->
// PX4-normalized pseudo-inverse pipeline; runs once at configuration time.
template <std::floating_point Scalar, std::size_t N>
ControlAllocatorCfg<Scalar, static_cast<int>(N)> MakeControlAllocatorCfg(
    const std::array<Rotor<Scalar>, N>& rotors) {
  constexpr int kNumMotors = static_cast<int>(N);
  Eigen::Matrix<Scalar, kNumMotors, 3> geometry;
  for (int i = 0; i < kNumMotors; ++i) {
    geometry.row(i) << rotors[i].arm_x, rotors[i].arm_y, rotors[i].km;
  }
  ControlAllocatorCfg<Scalar, kNumMotors> cfg;
  cfg.mix = NormalizedMix(RotorEffectiveness(geometry));
  return cfg;
}

// Stateless control-allocation kernel: distributes the body torque + collective
// thrust across the motors via the pre-normalized mix, then desaturates by an
// equal collective shift before clamping to [motor_min, motor_max]. Shifting
// all motors by a constant produces no net torque on a symmetric frame (each
// torque row of the effectiveness sums to zero), so the shift preserves
// roll/pitch/yaw authority and sacrifices only absolute thrust -- the "airmode"
// behaviour. This is a deliberate single-pass simplification of PX4's
// sequential, per-axis desaturation (no edge-case handling, per the project's
// opinionated ethos).
//
// Holds only its Cfg; allocate() is pure and const (like every apl20 kernel).
template <std::floating_point Scalar, int NumMotors>
class ControlAllocator {
 public:
  using Motors = Eigen::Matrix<Scalar, NumMotors, 1>;

  explicit ControlAllocator(const ControlAllocatorCfg<Scalar, NumMotors>& cfg)
      : cfg_(cfg) {}

  const ControlAllocatorCfg<Scalar, NumMotors>& cfg() const { return cfg_; }

  // `torque` is the normalized body torque [roll, pitch, yaw]; `collective` the
  // normalized collective thrust [0,1]. Returns per-motor normalized thrust in
  // [motor_min, motor_max], in the geometry's motor order.
  Motors allocate(const Eigen::Vector3<Scalar>& torque,
                  Scalar collective) const {
    Eigen::Vector4<Scalar> wrench;
    wrench << torque, collective;
    Motors motors = cfg_.mix * wrench;

    // Airmode desaturation: pull the most-saturated motor back to its rail by
    // shifting every motor equally (torque-neutral), then clamp the remainder.
    const Scalar hi = motors.maxCoeff();
    const Scalar lo = motors.minCoeff();
    if (hi > cfg_.motor_max) {
      motors.array() -= (hi - cfg_.motor_max);
    } else if (lo < cfg_.motor_min) {
      motors.array() += (cfg_.motor_min - lo);
    }
    return motors.cwiseMax(cfg_.motor_min).cwiseMin(cfg_.motor_max);
  }

 private:
  ControlAllocatorCfg<Scalar, NumMotors> cfg_;
};

}  // namespace apl

#endif  // APL_CONTROL_ALLOCATOR_HPP_
