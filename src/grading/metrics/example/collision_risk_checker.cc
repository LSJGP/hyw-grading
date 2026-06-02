#include "src/grading/metrics/example/collision_risk_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "src/grading/metric_register.h"
#include "spdlog/spdlog.h"

namespace grading_mini {
namespace {

constexpr double kWarnTtcS = 3.0;
constexpr double kCriticalTtcS = 1.5;
constexpr double kNearClearanceM = 0.5;

double SafeEgoLength(const MetricFrameInput& input) {
  return input.ego_vehicle().length() > 0.0 ? input.ego_vehicle().length() : 4.5;
}

double SafeEgoWidth(const MetricFrameInput& input) {
  return input.ego_vehicle().width() > 0.0 ? input.ego_vehicle().width() : 1.85;
}

struct PairRisk {
  bool overlap = false;
  double clearance_m = std::numeric_limits<double>::infinity();
  double ttc_s = std::numeric_limits<double>::infinity();
  double closing_speed_mps = 0.0;
};

PairRisk EvaluateRiskForNpc(const MetricFrameInput& input, const proto::NpcState& n) {
  PairRisk out;
  const auto& ego = input.vehicle_state();

  const double ego_length = SafeEgoLength(input);
  const double ego_width = SafeEgoWidth(input);
  const double ego_hl = ego_length * 0.5;
  const double ego_hw = ego_width * 0.5;

  const double npc_hl = std::max(0.5, n.length() * 0.5);
  const double npc_hw = std::max(0.3, n.width() * 0.5);

  const double dx_w = n.x() - ego.x();
  const double dy_w = n.y() - ego.y();
  const double c = std::cos(-ego.heading());
  const double s = std::sin(-ego.heading());
  const double dx = c * dx_w - s * dy_w;
  const double dy = s * dx_w + c * dy_w;

  const double x_sep = std::fabs(dx) - (ego_hl + npc_hl);
  const double y_sep = std::fabs(dy) - (ego_hw + npc_hw);
  const double x_gap = std::max(0.0, x_sep);
  const double y_gap = std::max(0.0, y_sep);
  out.clearance_m = std::hypot(x_gap, y_gap);
  out.overlap = (x_sep <= 0.0 && y_sep <= 0.0);

  if (out.overlap) {
    out.ttc_s = 0.0;
    return out;
  }

  const double ego_vx = ego.speed() * std::cos(ego.heading());
  const double ego_vy = ego.speed() * std::sin(ego.heading());
  const double rvx_w = n.vx() - ego_vx;
  const double rvy_w = n.vy() - ego_vy;
  const double rvx = c * rvx_w - s * rvy_w;
  const double rvy = s * rvx_w + c * rvy_w;

  const double center_dist = std::hypot(dx, dy);
  if (center_dist < 1e-6) {
    out.ttc_s = 0.0;
    out.closing_speed_mps = std::hypot(rvx, rvy);
    return out;
  }

  const double closing =
      -((dx / center_dist) * rvx + (dy / center_dist) * rvy);
  out.closing_speed_mps = std::max(0.0, closing);

  if (out.closing_speed_mps > 1e-3) {
    out.ttc_s = out.clearance_m / out.closing_speed_mps;
  }
  return out;
}

bool IsDangerous(const PairRisk& r) {
  if (r.overlap) return true;
  if (r.ttc_s < kCriticalTtcS) return true;
  if (r.ttc_s < kWarnTtcS && r.clearance_m < 1.0) return true;
  if (r.clearance_m < kNearClearanceM && r.closing_speed_mps > 0.5) return true;
  return false;
}

}  // namespace

REGISTER_METRIC(CollisionRiskChecker, "collision_risk_checker");

absl::Status CollisionRiskChecker::Init(
    const google::protobuf::Message* /*config*/) {
  SPDLOG_INFO(
      "CollisionRiskChecker init: warn_ttc={:.1f}s critical_ttc={:.1f}s",
      kWarnTtcS, kCriticalTtcS);
  return absl::OkStatus();
}

absl::Status CollisionRiskChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;
  if (input.npcs().empty()) {
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  PairRisk worst;
  bool has_eval = false;
  for (const auto& n : input.npcs()) {
    const PairRisk r = EvaluateRiskForNpc(input, n);
    if (!has_eval || r.ttc_s < worst.ttc_s ||
        (std::fabs(r.ttc_s - worst.ttc_s) < 1e-6 &&
         r.clearance_m < worst.clearance_m)) {
      worst = r;
      has_eval = true;
    }
  }
  if (!has_eval) {
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  min_ttc_s_ = std::min(min_ttc_s_, worst.ttc_s);
  min_clearance_m_ = std::min(min_clearance_m_, worst.clearance_m);

  const bool dangerous = IsDangerous(worst);
  if (worst.overlap) overlap_frames_++;
  if (worst.ttc_s < kWarnTtcS) imminent_frames_++;
  if (dangerous) risky_frames_++;

  output->set_bool_value(!dangerous);

  if (dangerous) {
    SPDLOG_WARN(
        "Frame {}: collision risk fail ttc={:.2f}s clearance={:.2f}m "
        "closing={:.2f}m/s overlap={}",
        input.frame_id(), worst.ttc_s, worst.clearance_m,
        worst.closing_speed_mps, worst.overlap ? "Y" : "n");
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> CollisionRiskChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(risky_frames_ == 0);

  const double min_ttc =
      (min_ttc_s_ >= 1e8) ? std::numeric_limits<double>::infinity() : min_ttc_s_;
  const double min_clearance =
      (min_clearance_m_ >= 1e8) ? std::numeric_limits<double>::infinity()
                                : min_clearance_m_;

  summary.set_detail(
      "risky_frames=" + std::to_string(risky_frames_) + "/" +
      std::to_string(total_frames_) + " overlap_frames=" +
      std::to_string(overlap_frames_) + " imminent_frames=" +
      std::to_string(imminent_frames_) + " min_ttc_s=" +
      (std::isfinite(min_ttc) ? std::to_string(min_ttc) : std::string("inf")) +
      " min_clearance_m=" +
      (std::isfinite(min_clearance) ? std::to_string(min_clearance)
                                    : std::string("inf")));
  return summary;
}

}  // namespace grading_mini
