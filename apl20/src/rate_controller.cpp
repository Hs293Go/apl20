#include "apl/rate_controller.hpp"

#include <algorithm>

namespace apl {

RateController::RateController(const RateControllerCfg& cfg)
    : cfg_(cfg), pid_(cfg.pid) {
  sat_hi_.setZero();
  sat_lo_.setZero();
}

void RateController::reset(const Eigen::Ref<const Eigen::Vector3d>& rate_meas) {
  pid_.reset(state_, rate_meas.array());
  sat_hi_.setZero();
  sat_lo_.setZero();
  terms_ = PidTerms<double, 3>{};
}

Eigen::Vector3d RateController::update(
    const Eigen::Ref<const Eigen::Vector3d>& rate_sp,
    const Eigen::Ref<const Eigen::Vector3d>& rate_meas, double throttle,
    double dt) {
  dt = std::clamp(dt, cfg_.dt_min, cfg_.dt_max);

  // Throttle PID attenuation (Betaflight TPA): above the breakpoint the same
  // P/D gains that are crisp at hover start to ring, so bleed them off linearly
  // with throttle. This rides on the kernel's pd_scale hook and leaves I alone.
  double tpa = 1.0;
  if (cfg_.tpa_breakpoint < 1.0 && throttle > cfg_.tpa_breakpoint) {
    const double frac =
        (throttle - cfg_.tpa_breakpoint) / (1.0 - cfg_.tpa_breakpoint);
    tpa = 1.0 - cfg_.tpa_rate * std::clamp(frac, 0.0, 1.0);
  }

  PidModifiers<double, 3> mods;
  mods.pd_scale.setConstant(tpa);
  // Feed back last step's clipping so the kernel freezes the wound-up
  // direction.
  mods.sat_hi = sat_hi_;
  mods.sat_lo = sat_lo_;

  terms_ = pid_.update(state_, rate_sp.array(), rate_meas.array(), dt, mods);

  const Eigen::Array3d raw = terms_.output();
  // The output clamp is our saturation source: an axis that clips here is, by
  // definition, the actuator running out of authority. Latch the direction for
  // next step's conditional integration, then return the clamped command.
  sat_hi_ = raw > cfg_.output_limit;
  sat_lo_ = raw < -cfg_.output_limit;
  return raw.max(-cfg_.output_limit).min(cfg_.output_limit).matrix();
}

}  // namespace apl
