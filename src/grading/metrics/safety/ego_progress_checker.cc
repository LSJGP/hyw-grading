#include "src/grading/metrics/safety/ego_progress_checker.h"

#include <cmath>
#include <sstream>

#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"
#include "src/grading/metrics/safety/geometry.h"

namespace grading_mini {
namespace {

constexpr double kRouteLenEpsilon = 1e-3;
constexpr double kTimeEpsilonS = 1e-3;

double Clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

double RouteLengthM(const proto::RoutePose2D& start, const proto::RoutePose2D& end) {
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  return std::hypot(dx, dy);
}

double ComputeEfficiencyScore(double along_ratio, bool reached_end_line,
                              double initial_along_ratio, double elapsed_s,
                              double sim_duration_s) {
  if (sim_duration_s <= kTimeEpsilonS || elapsed_s <= kTimeEpsilonS) {
    return 1.0;
  }
  if (reached_end_line) {
    return Clamp01(sim_duration_s / elapsed_s);
  }
  const double progress = std::max(0.0, along_ratio - initial_along_ratio);
  const double expected_progress = Clamp01(elapsed_s / sim_duration_s);
  if (expected_progress <= 1e-6) {
    return 1.0;
  }
  return Clamp01(progress / expected_progress);
}

double CombineScores(double position_score, double efficiency_score,
                     double efficiency_weight) {
  const double w = Clamp01(efficiency_weight);
  return position_score * (1.0 - w + w * Clamp01(efficiency_score));
}

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
      if (typed->has_efficiency_weight()) {
        efficiency_weight_ = std::min(1.0, std::max(0.0, typed->efficiency_weight()));
      }
    }
  }
  SPDLOG_INFO(
      "EgoProgressChecker init: progress_exponent={:.2f} pass_threshold={:.2f} "
      "efficiency_weight={:.2f}",
      progress_exponent_, pass_threshold_, efficiency_weight_);
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
    route_length_m_ = RouteLengthM(sdc_route_.start(), sdc_route_.end());
    if (route_length_m_ < kRouteLenEpsilon) {
      route_invalid_ = true;
    }
  }

  if (input.has_scenario_context()) {
    scenario_context_ = input.scenario_context();
    has_scenario_context_ = true;
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
  double position_score = 0.0;
  bool reached = false;
  const auto& vs = input.vehicle_state();
  if (!ComputeFrameScore(sdc_route_.start(), sdc_route_.end(), vs.x(), vs.y(),
                         &signed_m, &along, &position_score, &reached)) {
    if (!route_invalid_) {
      SPDLOG_WARN("EgoProgressChecker: degenerate SDC route (start ~= end)");
      route_invalid_ = true;
    }
    skipped_frames_++;
    output->set_bool_value(true);
    output->set_double_value(0.0);
    return absl::OkStatus();
  }

  if (!has_progress_origin_) {
    first_timestamp_us_ = input.timestamp_us();
    initial_along_ratio_ = along;
    has_progress_origin_ = true;
  }

  const double elapsed_s =
      first_timestamp_us_ >= 0
          ? std::max(0.0, static_cast<double>(input.timestamp_us() -
                                              first_timestamp_us_) /
                               1e6)
          : 0.0;

  double efficiency_score = 1.0;
  double sim_duration_s = 0.0;
  if (has_scenario_context_ && scenario_context_.sim_duration_s() > kTimeEpsilonS) {
    sim_duration_s = scenario_context_.sim_duration_s();
    efficiency_score = ComputeEfficiencyScore(
        along, reached, initial_along_ratio_, elapsed_s, sim_duration_s);
  } else if (efficiency_weight_ > 0.0 && !warned_no_scenario_context_) {
    SPDLOG_WARN(
        "EgoProgressChecker: missing scenario_context.sim_duration_s; "
        "efficiency defaults to 1.0 until frame 0 provides it");
    warned_no_scenario_context_ = true;
  }

  const double combined_score =
      CombineScores(position_score, efficiency_score, efficiency_weight_);

  evaluated_frames_++;
  max_frame_score_ = std::max(max_frame_score_, combined_score);
  final_frame_score_ = position_score;
  final_combined_score_ = combined_score;
  final_efficiency_score_ = efficiency_score;
  max_signed_progress_m_ = std::max(max_signed_progress_m_, signed_m);

  output->set_bool_value(reached);
  output->set_double_value(combined_score);

  proto::EgoProgressCheckerCustomInfo info;
  info.set_signed_progress_m(signed_m);
  info.set_along_ratio(along);
  info.set_frame_score(position_score);
  info.set_reached_end_line(reached);
  info.set_efficiency_score(efficiency_score);
  info.set_combined_frame_score(combined_score);
  info.set_sim_duration_s(sim_duration_s);
  output->mutable_custom_info()->PackFrom(info);

  if (reached) {
    SPDLOG_INFO(
        "Frame {}: ego reached SDC end line (signed_progress={:.2f}m "
        "efficiency={:.3f} combined={:.3f} sim_duration={:.2f}s)",
        input.frame_id(), signed_m, efficiency_score, combined_score,
        sim_duration_s);
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

  summary.set_passed(final_combined_score_ >= pass_threshold_);

  std::ostringstream oss;
  oss << "final_combined=" << final_combined_score_
      << " final_position=" << final_frame_score_
      << " final_efficiency=" << final_efficiency_score_
      << " max_combined=" << max_frame_score_
      << " pass_threshold=" << pass_threshold_
      << " efficiency_weight=" << efficiency_weight_;
  if (has_scenario_context_) {
    oss << " sim_duration_s=" << scenario_context_.sim_duration_s()
        << " scenario_duration_s=" << scenario_context_.scenario_duration_s();
  }
  oss << " max_signed_progress_m=" << max_signed_progress_m_
      << " evaluated_frames=" << evaluated_frames_ << "/"
      << total_frames_;
  if (skipped_frames_ > 0) {
    oss << " skipped_frames=" << skipped_frames_;
  }
  summary.set_detail(oss.str());
  return summary;
}

}  // namespace grading_mini
