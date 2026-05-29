#include "src/grading/metrics/safety/solid_line_crossing_checker.h"

#include <cmath>
#include <limits>

#include "spdlog/spdlog.h"
#include "src/grading/metrics/safety/geometry.h"

namespace grading_mini {

REGISTER_METRIC(SolidLineCrossingChecker, "solid_line_crossing_checker");

bool SolidLineCrossingChecker::IsSolidLineType(const std::string& type) {
  return type == "TYPE_SOLID_SINGLE_WHITE" ||
         type == "TYPE_SOLID_DOUBLE_WHITE" ||
         type == "TYPE_SOLID_SINGLE_YELLOW" ||
         type == "TYPE_SOLID_DOUBLE_YELLOW" ||
         type == "TYPE_PASSING_DOUBLE_YELLOW";
}

double SolidLineCrossingChecker::MinDistToSolidLines(
    const proto::SceneMap& map, double x, double y) const {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& line : map.road_lines()) {
    if (!IsSolidLineType(line.type())) continue;
    if (line.polyline_size() < 2) continue;
    for (int i = 0; i + 1 < line.polyline_size(); ++i) {
      const auto& p0 = line.polyline(i);
      const auto& p1 = line.polyline(i + 1);
      best = std::min(best, PointToSegmentDist(x, y, p0.x(), p0.y(), p1.x(), p1.y()));
    }
  }
  return best;
}

double SolidLineCrossingChecker::MinDistEgoObbToSolidLines(
    const MetricFrameInput& input) const {
  double min_dist = std::numeric_limits<double>::infinity();
  const Obb2D box = MakeEgoObb(input);
  for (const auto& corner : ObbCorners(box)) {
    min_dist = std::min(min_dist,
                        MinDistToSolidLines(scene_map_, corner[0], corner[1]));
  }
  return min_dist;
}

absl::Status SolidLineCrossingChecker::Init(
    const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::SolidLineCrossingCheckerConfig*>(config)) {
      if (typed->min_clearance_m() > 0.0) {
        min_clearance_m_ = typed->min_clearance_m();
      }
    }
  }
  SPDLOG_INFO("SolidLineCrossingChecker: clearance={:.2f}m", min_clearance_m_);
  return absl::OkStatus();
}

absl::Status SolidLineCrossingChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (input.has_scene_map()) {
    scene_map_ = input.scene_map();
    has_scene_map_ = true;
  }

  if (!has_scene_map_ || scene_map_.road_lines_size() == 0) {
    if (!warned_no_map_) {
      SPDLOG_WARN("SolidLineCrossingChecker: no scene_map, skip");
      warned_no_map_ = true;
    }
    skipped_frames_++;
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  const double min_dist = MinDistEgoObbToSolidLines(input);
  last_min_dist_m_ = min_dist;
  const bool ok = !(std::isfinite(min_dist) && min_dist < min_clearance_m_);
  if (!ok) {
    violation_frames_++;
    SPDLOG_WARN("Frame {}: solid line violation min_dist={:.2f}m",
                input.frame_id(), min_dist);
  }

  output->set_bool_value(ok);
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> SolidLineCrossingChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_frames_ == 0);
  summary.set_detail(
      "violation_frames=" + std::to_string(violation_frames_) + "/" +
      std::to_string(total_frames_) +
      " skipped=" + std::to_string(skipped_frames_) +
      " last_min_dist_m=" + std::to_string(last_min_dist_m_));
  return summary;
}

}  // namespace grading_mini
