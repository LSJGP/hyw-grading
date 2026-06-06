#pragma once

#include "proto/grading/metrics/safety_metric.pb.h"
#include "proto/grading/scene.pb.h"
#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class SolidLineCrossingChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  proto::SceneMap scene_map_;
  bool has_scene_map_ = false;
  bool warned_no_map_ = false;

  int total_frames_ = 0;
  int violation_frames_ = 0;
  int skipped_frames_ = 0;
  bool last_violation_ = false;
};

}  // namespace grading_mini
