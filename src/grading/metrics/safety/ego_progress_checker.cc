#include "src/grading/metrics/safety/ego_progress_checker.h"

#include <cmath>
#include <sstream>

#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"
#include "src/grading/metrics/safety/geometry.h"

namespace grading_mini {
namespace {

constexpr double kRouteLenEpsilon = 1e-3;

double Clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

}  // namespace

REGISTER_METRIC(EgoProgressChecker, "ego_progress_checker");

absl::Status EgoProgressChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::EgoProgressCheckerConfig*>(config)) {
      if (typed->progress_exponent() > 0.0) {
        progress_exponent_ = typed->progress_exponent();
      }
      if (typed->pass_threshold() > 0.0) {
        pass_threshold_ = typed->pass_threshold();
      }
    }
  }
  SPDLOG_INFO(
      "EgoProgressChecker init: progress_exponent={:.2f} pass_threshold={:.2f}",
      progress_exponent_, pass_threshold_);
  return absl::OkStatus();
}

bool EgoProgressChecker::ComputeFrameScore(const proto::RoutePose2D& start,
                                           const proto::RoutePose2D& end,
                                           double ego_x, double ego_y,
                                           double* signed_progress_m,
                                           double* along_ratio,
                                           double* frame_score,
                                           bool* reached_end_line) const {
  const double vx = end.x() - start.x();
  const double vy = end.y() - start.y();
  const double route_len_sq = vx * vx + vy * vy;
  if (route_len_sq < kRouteLenEpsilon * kRouteLenEpsilon) {
    return false;
  }

  const double forward_x = std::cos(end.yaw());
  const double forward_y = std::sin(end.yaw());
  const double rel_x = ego_x - end.x();
  const double rel_y = ego_y - end.y();

  const double signed_m = Dot2D(rel_x, rel_y, forward_x, forward_y);
  const double along =
      Dot2D(ego_x - start.x(), ego_y - start.y(), vx, vy) / route_len_sq;
  const double along_clamped = Clamp01(along);

  const bool reached = signed_m >= 0.0;
  const double score =
      reached ? 1.0 : std::pow(along_clamped, progress_exponent_);

  *signed_progress_m = signed_m;
  *along_ratio = along_clamped;
  *frame_score = score;
  *reached_end_line = reached;
  return true;
}

absl::Status EgoProgressChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (input.has_sdc_route()) {
    sdc_route_ = input.sdc_route();
    has_sdc_route_ = true;
    route_invalid_ = false;
  }

  if (!has_sdc_route_) {
    if (!warned_no_route_) {
      SPDLOG_WARN(
          "EgoProgressChecker: missing sdc_route; skipping frames until route is "
          "provided on frame 0");
      warned_no_route_ = true;
    }
    skipped_frames_++;
    output->set_bool_value(true);
    output->set_double_value(0.0);
    return absl::OkStatus();
  }

  double signed_m = 0.0;
  double along = 0.0;
  double score = 0.0;
  bool reached = false;
  const auto& vs = input.vehicle_state();
  if (!ComputeFrameScore(sdc_route_.start(), sdc_route_.end(), vs.x(), vs.y(),
                         &signed_m, &along, &score, &reached)) {
    if (!route_invalid_) {
      SPDLOG_WARN("EgoProgressChecker: degenerate SDC route (start ~= end)");
      route_invalid_ = true;
    }
    skipped_frames_++;
    output->set_bool_value(true);
    output->set_double_value(0.0);
    return absl::OkStatus();
  }

  evaluated_frames_++;
  max_frame_score_ = std::max(max_frame_score_, score);
  final_frame_score_ = score;
  max_signed_progress_m_ = std::max(max_signed_progress_m_, signed_m);

  output->set_bool_value(reached);
  output->set_double_value(score);

  proto::EgoProgressCheckerCustomInfo info;
  info.set_signed_progress_m(signed_m);
  info.set_along_ratio(along);
  info.set_frame_score(score);
  info.set_reached_end_line(reached);
  output->mutable_custom_info()->PackFrom(info);

  if (reached) {
    SPDLOG_INFO("Frame {}: ego reached SDC end line (signed_progress={:.2f}m)",
                input.frame_id(), signed_m);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> EgoProgressChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);

  if (!has_sdc_route_ || route_invalid_ || evaluated_frames_ == 0) {
    summary.set_passed(false);
    if (!has_sdc_route_) {
      summary.set_detail("missing_sdc_route");
    } else if (route_invalid_) {
      summary.set_detail("degenerate_sdc_route");
    } else {
      summary.set_detail("no_evaluated_frames");
    }
    return summary;
  }

  summary.set_passed(final_frame_score_ >= pass_threshold_);

  std::ostringstream oss;
  oss << "final_score=" << final_frame_score_ << " max_score=" << max_frame_score_
      << " pass_threshold=" << pass_threshold_
      << " max_signed_progress_m=" << max_signed_progress_m_
      << " evaluated_frames=" << evaluated_frames_ << "/"
      << total_frames_;
  if (skipped_frames_ > 0) {
    oss << " skipped_frames=" << skipped_frames_;
  }
  summary.set_detail(oss.str());
  return summary;
}

}  // namespace grading_mini
