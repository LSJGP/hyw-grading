#include "src/grading/metrics/example/speed_checker.h"

#include "spdlog/spdlog.h"

namespace grading_mini {

REGISTER_METRIC(SpeedChecker, "speed_checker");

absl::Status SpeedChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    auto* typed = dynamic_cast<const proto::SpeedChecker*>(config);
    if (typed) {
      if (typed->max_speed_threshold() > 0) {
        max_speed_ = typed->max_speed_threshold();
      }
      if (typed->has_min_speed_threshold_mps()) {
        min_speed_ = typed->min_speed_threshold_mps();
      }
      if (typed->has_max_acceleration_mps2()) {
        max_acceleration_ = typed->max_acceleration_mps2();
      }
    }
  }
  SPDLOG_INFO(
      "SpeedChecker init: max_speed={:.1f} min_speed={:.1f} "
      "max_accel={:.1f} m/s",
      max_speed_, min_speed_, max_acceleration_);
  return absl::OkStatus();
}

absl::Status SpeedChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  const double speed = input.vehicle_state().speed();
  const double accel = input.vehicle_state().acceleration();

  const bool overspeed = speed > max_speed_;
  const bool braking = accel < -0.5;
  const bool nearly_stopped = speed < 0.3;
  const bool too_slow =
      min_speed_ > 0.0 && speed < min_speed_ && !braking && !nearly_stopped;
  const bool excessive_accel =
      max_acceleration_ > 0.0 && accel > max_acceleration_;

  if (overspeed) {
    overspeed_violations_++;
  }
  if (too_slow) {
    too_slow_violations_++;
  }
  if (excessive_accel) {
    excessive_accel_violations_++;
  }
  total_frames_++;

  const bool passed = !overspeed && !too_slow && !excessive_accel;
  output->set_bool_value(passed);

  proto::SpeedCheckerCustomInfo info;
  info.set_current_speed(speed);
  info.set_exceeded(overspeed);
  info.set_current_acceleration(accel);
  info.set_too_slow(too_slow);
  info.set_excessive_acceleration(excessive_accel);
  output->mutable_custom_info()->PackFrom(info);

  if (overspeed) {
    SPDLOG_WARN("Frame {}: speed {:.1f} > {:.1f}", input.frame_id(), speed,
                max_speed_);
  }
  if (too_slow) {
    SPDLOG_WARN("Frame {}: speed {:.1f} < {:.1f} (cruising too slow)",
                input.frame_id(), speed, min_speed_);
  }
  if (excessive_accel) {
    SPDLOG_WARN("Frame {}: accel {:.2f} > {:.1f}", input.frame_id(), accel,
                max_acceleration_);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> SpeedChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  const int violations =
      overspeed_violations_ + too_slow_violations_ + excessive_accel_violations_;
  summary.set_passed(violations == 0);
  summary.set_detail("violations=" + std::to_string(violations) + "/" +
                     std::to_string(total_frames_) +
                     " overspeed=" + std::to_string(overspeed_violations_) +
                     " too_slow=" + std::to_string(too_slow_violations_) +
                     " excessive_accel=" +
                     std::to_string(excessive_accel_violations_));
  return summary;
}

}  // namespace grading_mini
