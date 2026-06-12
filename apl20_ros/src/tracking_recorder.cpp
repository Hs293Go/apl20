#include "apl20_ros/tracking_recorder.hpp"

#include <cmath>

namespace apl20_ros {

TrackingRecorder::TrackingRecorder(const std::string& path) {
  if (path.empty()) {
    return;  // recording disabled
  }
  file_.open(path);
  if (file_.is_open()) {
    file_ << "t,wp,ref_n,ref_e,ref_d,ref_yaw,pos_n,pos_e,pos_d,yaw,pos_err,"
             "yaw_err\n";
  }
}

void TrackingRecorder::record(double t, int wp, const Eigen::Vector3d& ref_pos,
                              double ref_yaw, const Eigen::Vector3d& pos,
                              double yaw) {
  if (!file_.is_open()) {
    return;
  }
  const double pos_err = (ref_pos - pos).norm();
  const double yaw_err = std::remainder(ref_yaw - yaw, 2.0 * M_PI);
  file_ << t << ',' << wp << ',' << ref_pos.x() << ',' << ref_pos.y() << ','
        << ref_pos.z() << ',' << ref_yaw << ',' << pos.x() << ',' << pos.y()
        << ',' << pos.z() << ',' << yaw << ',' << pos_err << ',' << yaw_err
        << '\n';
}

}  // namespace apl20_ros
