#ifndef APL20_ROS_TRACKING_RECORDER_HPP_
#define APL20_ROS_TRACKING_RECORDER_HPP_

#include <Eigen/Core>
#include <fstream>
#include <string>

namespace apl20_ros {

// Records waypoint-tracking performance to a CSV file for offline analysis.
// Each row is one control cycle: the shaped reference the controller is
// tracking, the measured pose, and the position/heading errors -- enough to
// compute per-leg RMS error, settling time, overshoot, etc. NED throughout.
//
// An empty path disables recording (enabled() == false), so the node can always
// construct one and call record() unconditionally.
class TrackingRecorder {
 public:
  explicit TrackingRecorder(const std::string& path);

  bool enabled() const { return file_.is_open(); }

  // One sample. `t` is seconds since flight start; `wp` the active waypoint
  // index; `ref_pos`/`ref_yaw` the shaped reference; `pos`/`yaw` the measured
  // pose.
  void record(double t, int wp, const Eigen::Vector3d& ref_pos, double ref_yaw,
              const Eigen::Vector3d& pos, double yaw);

 private:
  std::ofstream file_;
};

}  // namespace apl20_ros

#endif  // APL20_ROS_TRACKING_RECORDER_HPP_
