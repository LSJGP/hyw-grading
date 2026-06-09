#include "src/grading/metrics/safety/hard_braking_checker.h"

#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"

namespace grading_mini {

REGISTER_METRIC(HardBrakingChecker, "hard_braking_checker");

absl::Status HardBrakingChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::HardBrakingCheckerConfig*>(config)) {
      if (typed->min_decel_threshold_mps2() > 0.0) {
        min_decel_threshold_mps2_ = typed->min_decel_threshold_mps2();
      }
      if (typed->min_speed_mps() >= 0.0) {
        min_speed_mps_ = typed->min_speed_mps();
      }
    }
  }
  SPDLOG_INFO(
      "HardBrakingChecker init: decel_threshold={:.1f} m/s^2 min_speed={:.1f} "
      "m/s",
      min_decel_threshold_mps2_, min_speed_mps_);
  return absl::OkStatus();
}

absl::Status HardBrakingChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  const double accel = input.vehicle_state().acceleration();
  const double speed = input.vehicle_state().speed();
  const bool hard_braking =
      speed >= min_speed_mps_ && accel < -min_decel_threshold_mps2_;

  total_frames_++;
  if (hard_braking) {
    violation_frames_++;
  }

  output->set_bool_value(!hard_braking);

  proto::HardBrakingCheckerCustomInfo info;
  info.set_current_acceleration(accel);
  info.set_current_speed(speed);
  info.set_hard_braking(hard_braking);
  output->mutable_custom_info()->PackFrom(info);

  if (hard_braking) {
    SPDLOG_WARN("Frame {}: hard braking accel {:.2f} m/s^2 (threshold -{:.1f})",
                input.frame_id(), accel, min_decel_threshold_mps2_);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> HardBrakingChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_frames_ == 0);
  summary.set_detail("violation_frames=" + std::to_string(violation_frames_) +
                     "/" + std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
