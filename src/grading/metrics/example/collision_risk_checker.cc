#include "src/grading/metrics/example/collision_risk_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "proto/grading/metrics/example_metric.pb.h"
#include "src/grading/metric_register.h"
#include "spdlog/spdlog.h"

namespace grading_mini {
namespace {

constexpr double kDefaultWarnTtcS = 3.0;
constexpr double kDefaultCriticalTtcS = 1.5;
constexpr double kDefaultNearClearanceM = 0.5;
constexpr double kDefaultMaxLongitudinalM = 40.0;
constexpr double kDefaultMaxLateralM = 1.2;
constexpr double kDefaultMinClosingSpeedMps = 0.5;
constexpr double kDefaultMinForwardCos = 0.9396926207859083;  // cos(20 deg)
constexpr double kDefaultPathLateralTolM = 1.0;
constexpr double kDefaultPathNpcWidthRatio = 0.25;

double SafeEgoLength(const MetricFrameInput& input) {
  return input.ego_vehicle().length() > 0.0 ? input.ego_vehicle().length() : 4.5;
}

double SafeEgoWidth(const MetricFrameInput& input) {
  return input.ego_vehicle().width() > 0.0 ? input.ego_vehicle().width() : 1.85;
}

struct EgoLocalOffset {
  double dx = 0.0;
  double dy = 0.0;
};

struct PathProjection {
  double arc_m = 0.0;
  double lateral_m = std::numeric_limits<double>::infinity();
};

EgoLocalOffset ToEgoLocal(const MetricFrameInput& input, const proto::NpcState& n) {
  const auto& ego = input.vehicle_state();
  const double dx_w = n.x() - ego.x();
  const double dy_w = n.y() - ego.y();
  const double c = std::cos(-ego.heading());
  const double s = std::sin(-ego.heading());
  return {c * dx_w - s * dy_w, s * dx_w + c * dy_w};
}

PathProjection ProjectOntoPlannedPath(double x, double y,
                                    const proto::PlannedTrajectory& traj) {
  PathProjection best;
  double arc = 0.0;
  const int n_pts = traj.points_size();
  for (int i = 0; i < n_pts - 1; ++i) {
    const auto& p0 = traj.points(i);
    const auto& p1 = traj.points(i + 1);
    const double dx = p1.x() - p0.x();
    const double dy = p1.y() - p0.y();
    const double seg_len2 = dx * dx + dy * dy;
    double t = 0.0;
    double px = p0.x();
    double py = p0.y();
    double seg_len = 0.0;
    if (seg_len2 > 1e-12) {
      seg_len = std::sqrt(seg_len2);
      t = std::clamp(((x - p0.x()) * dx + (y - p0.y()) * dy) / seg_len2, 0.0,
                     1.0);
      px = p0.x() + t * dx;
      py = p0.y() + t * dy;
    }
    const double lat = std::hypot(x - px, y - py);
    const double arc_at = arc + t * seg_len;
    if (lat < best.lateral_m) {
      best.lateral_m = lat;
      best.arc_m = arc_at;
    }
    arc += seg_len;
  }
  return best;
}

bool HasPlannedPath(const MetricFrameInput& input) {
  return input.has_planned_trajectory() &&
         input.planned_trajectory().points_size() >= 2;
}

struct PairRisk {
  bool overlap = false;
  double clearance_m = std::numeric_limits<double>::infinity();
  double ttc_s = std::numeric_limits<double>::infinity();
  double closing_speed_mps = 0.0;
};

bool PassesPlannedPathFilter(const MetricFrameInput& input,
                             const proto::NpcState& n,
                             bool require_planned_path, double path_lateral_tol_m,
                             double path_npc_width_ratio, double max_longitudinal_m,
                             double ego_half_length) {
  if (!require_planned_path || !HasPlannedPath(input)) {
    return true;
  }
  const auto& ego = input.vehicle_state();
  const auto& traj = input.planned_trajectory();
  const PathProjection ego_proj =
      ProjectOntoPlannedPath(ego.x(), ego.y(), traj);
  const PathProjection npc_proj = ProjectOntoPlannedPath(n.x(), n.y(), traj);

  const double lateral_tol = path_lateral_tol_m + path_npc_width_ratio * n.width();
  if (npc_proj.lateral_m > lateral_tol) {
    return false;
  }

  const double npc_half_length = std::max(0.5, n.length() * 0.5);
  const double path_gap =
      npc_proj.arc_m - ego_proj.arc_m - ego_half_length - npc_half_length;
  if (path_gap < -0.5) {
    return false;
  }
  if (path_gap > max_longitudinal_m) {
    return false;
  }
  return true;
}

bool PassesForwardCorridorFilter(double dx, double dy, double npc_width,
                                 bool require_forward_corridor,
                                 double max_longitudinal_m, double max_lateral_m,
                                 double path_npc_width_ratio) {
  if (!require_forward_corridor) {
    return true;
  }
  if (dx <= 0.0 || dx > max_longitudinal_m) {
    return false;
  }
  const double lateral_tol = max_lateral_m + path_npc_width_ratio * npc_width;
  return std::fabs(dy) <= lateral_tol;
}

bool PassesApproachHeadingFilter(double dx, double center_dist, double closing_speed,
                                 bool overlap, bool require_approach_heading,
                                 double min_closing_speed_mps,
                                 double min_forward_cos) {
  if (!require_approach_heading || overlap) {
    return true;
  }
  if (closing_speed <= min_closing_speed_mps) {
    return false;
  }
  if (center_dist < 1e-6) {
    return true;
  }
  return (dx / center_dist) >= min_forward_cos;
}

bool PassesSpatialFilters(const MetricFrameInput& input, const proto::NpcState& n,
                          const EgoLocalOffset& local, bool overlap,
                          bool require_planned_path, double path_lateral_tol_m,
                          double path_npc_width_ratio, double max_longitudinal_m,
                          double ego_half_length, bool require_forward_corridor,
                          double max_lateral_m) {
  if (overlap) {
    return true;
  }
  if (HasPlannedPath(input) && require_planned_path) {
    return PassesPlannedPathFilter(input, n, require_planned_path,
                                   path_lateral_tol_m, path_npc_width_ratio,
                                   max_longitudinal_m, ego_half_length);
  }
  return PassesForwardCorridorFilter(local.dx, local.dy, n.width(),
                                     require_forward_corridor, max_longitudinal_m,
                                     max_lateral_m, path_npc_width_ratio);
}

std::optional<PairRisk> EvaluateRiskForNpc(const MetricFrameInput& input,
                                          const proto::NpcState& n,
                                          bool require_planned_path,
                                          double path_lateral_tol_m,
                                          double path_npc_width_ratio,
                                          bool require_forward_corridor,
                                          double max_longitudinal_m,
                                          double max_lateral_m,
                                          bool require_approach_heading,
                                          double min_closing_speed_mps,
                                          double min_forward_cos) {
  const auto& ego = input.vehicle_state();
  const EgoLocalOffset local = ToEgoLocal(input, n);

  const double ego_length = SafeEgoLength(input);
  const double ego_width = SafeEgoWidth(input);
  const double ego_hl = ego_length * 0.5;
  const double ego_hw = ego_width * 0.5;

  const double npc_hl = std::max(0.5, n.length() * 0.5);
  const double npc_hw = std::max(0.3, n.width() * 0.5);

  const double x_sep = std::fabs(local.dx) - (ego_hl + npc_hl);
  const double y_sep = std::fabs(local.dy) - (ego_hw + npc_hw);
  const double x_gap = std::max(0.0, x_sep);
  const double y_gap = std::max(0.0, y_sep);
  PairRisk out;
  out.clearance_m = std::hypot(x_gap, y_gap);
  out.overlap = (x_sep <= 0.0 && y_sep <= 0.0);

  if (!PassesSpatialFilters(input, n, local, out.overlap, require_planned_path,
                            path_lateral_tol_m, path_npc_width_ratio,
                            max_longitudinal_m, ego_hl, require_forward_corridor,
                            max_lateral_m)) {
    return std::nullopt;
  }

  if (out.overlap) {
    out.ttc_s = 0.0;
    return out;
  }

  const double ego_vx = ego.speed() * std::cos(ego.heading());
  const double ego_vy = ego.speed() * std::sin(ego.heading());
  const double c = std::cos(-ego.heading());
  const double s = std::sin(-ego.heading());
  const double rvx_w = n.vx() - ego_vx;
  const double rvy_w = n.vy() - ego_vy;
  const double rvx = c * rvx_w - s * rvy_w;
  const double rvy = s * rvx_w + c * rvy_w;

  const double center_dist = std::hypot(local.dx, local.dy);
  if (center_dist < 1e-6) {
    out.ttc_s = 0.0;
    out.closing_speed_mps = std::hypot(rvx, rvy);
    return out;
  }

  const double closing =
      -((local.dx / center_dist) * rvx + (local.dy / center_dist) * rvy);
  out.closing_speed_mps = std::max(0.0, closing);

  if (!PassesApproachHeadingFilter(local.dx, center_dist, out.closing_speed_mps,
                                   out.overlap, require_approach_heading,
                                   min_closing_speed_mps, min_forward_cos)) {
    return std::nullopt;
  }

  if (out.closing_speed_mps > 1e-3) {
    out.ttc_s = out.clearance_m / out.closing_speed_mps;
  }
  return out;
}

bool IsDangerous(const PairRisk& r, double warn_ttc_s, double critical_ttc_s,
                 double near_clearance_m) {
  if (r.overlap) return true;
  if (r.ttc_s < critical_ttc_s) return true;
  if (r.ttc_s < warn_ttc_s && r.clearance_m < 1.0) return true;
  if (r.clearance_m < near_clearance_m && r.closing_speed_mps > 0.5) return true;
  return false;
}

}  // namespace

REGISTER_METRIC(CollisionRiskChecker, "collision_risk_checker");

absl::Status CollisionRiskChecker::Init(
    const google::protobuf::Message* config) {
  warn_ttc_s_ = kDefaultWarnTtcS;
  critical_ttc_s_ = kDefaultCriticalTtcS;
  near_clearance_m_ = kDefaultNearClearanceM;
  require_forward_corridor_ = true;
  max_longitudinal_m_ = kDefaultMaxLongitudinalM;
  max_lateral_m_ = kDefaultMaxLateralM;
  require_approach_heading_ = true;
  min_closing_speed_mps_ = kDefaultMinClosingSpeedMps;
  min_forward_cos_ = kDefaultMinForwardCos;
  require_planned_path_ = true;
  path_lateral_tol_m_ = kDefaultPathLateralTolM;
  path_npc_width_ratio_ = kDefaultPathNpcWidthRatio;

  if (config) {
    const auto* typed =
        dynamic_cast<const proto::CollisionRiskCheckerConfig*>(config);
    if (typed) {
      if (typed->warn_ttc_s() > 0.0) warn_ttc_s_ = typed->warn_ttc_s();
      if (typed->critical_ttc_s() > 0.0) {
        critical_ttc_s_ = typed->critical_ttc_s();
      }
      if (typed->near_clearance_m() > 0.0) {
        near_clearance_m_ = typed->near_clearance_m();
      }
      require_forward_corridor_ = !typed->disable_forward_corridor();
      if (typed->max_longitudinal_m() > 0.0) {
        max_longitudinal_m_ = typed->max_longitudinal_m();
      }
      if (typed->max_lateral_m() > 0.0) {
        max_lateral_m_ = typed->max_lateral_m();
      }
      require_approach_heading_ = !typed->disable_approach_heading();
      if (typed->min_closing_speed_mps() > 0.0) {
        min_closing_speed_mps_ = typed->min_closing_speed_mps();
      }
      if (typed->min_forward_cos() > 0.0 && typed->min_forward_cos() <= 1.0) {
        min_forward_cos_ = typed->min_forward_cos();
      }
      require_planned_path_ = !typed->disable_planned_path();
      if (typed->path_lateral_tol_m() > 0.0) {
        path_lateral_tol_m_ = typed->path_lateral_tol_m();
      }
      if (typed->path_npc_width_ratio() > 0.0) {
        path_npc_width_ratio_ = typed->path_npc_width_ratio();
      }
    }
  }

  SPDLOG_INFO(
      "CollisionRiskChecker init: warn_ttc={:.1f}s critical_ttc={:.1f}s "
      "planned_path={} path_lat_tol={:.1f}m npc_width_ratio={:.2f} "
      "forward_corridor={} approach_heading={}",
      warn_ttc_s_, critical_ttc_s_, require_planned_path_, path_lateral_tol_m_,
      path_npc_width_ratio_, require_forward_corridor_, require_approach_heading_);
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
    const auto risk_or = EvaluateRiskForNpc(
        input, n, require_planned_path_, path_lateral_tol_m_,
        path_npc_width_ratio_, require_forward_corridor_, max_longitudinal_m_,
        max_lateral_m_, require_approach_heading_, min_closing_speed_mps_,
        min_forward_cos_);
    if (!risk_or.has_value()) {
      continue;
    }
    const PairRisk& r = *risk_or;
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

  const bool dangerous =
      IsDangerous(worst, warn_ttc_s_, critical_ttc_s_, near_clearance_m_);
  if (worst.overlap) overlap_frames_++;
  if (worst.ttc_s < warn_ttc_s_) imminent_frames_++;
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
