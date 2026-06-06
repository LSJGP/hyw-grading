#include "src/grading/metrics/safety/solid_line_crossing_checker.h"

#include "spdlog/spdlog.h"
#include "src/grading/metrics/safety/geometry.h"

namespace grading_mini {

REGISTER_METRIC(SolidLineCrossingChecker, "solid_line_crossing_checker");

absl::Status SolidLineCrossingChecker::Init(
    const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::SolidLineCrossingCheckerConfig*>(config)) {
      if (typed->min_clearance_m() > 0.0) {
        SPDLOG_WARN(
            "SolidLineCrossingChecker: min_clearance_m is deprecated "
            "(OBB edge intersection mode ignores it)");
      }
    }
  }
  SPDLOG_INFO("SolidLineCrossingChecker: OBB edge intersection mode");
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

  const bool crossed = EgoObbIntersectsSolidLines(input, scene_map_);
  last_violation_ = crossed;
  if (crossed) {
    violation_frames_++;
    SPDLOG_WARN("Frame {}: solid line crossing (OBB edge intersects)",
                input.frame_id());
  }

  output->set_bool_value(!crossed);
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
      " last_violation=" + (last_violation_ ? "Y" : "n"));
  return summary;
}

}  // namespace grading_mini
