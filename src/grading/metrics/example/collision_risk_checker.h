#pragma once

#include "src/grading/metric_base.h"

namespace grading_mini {

// Frame-level collision risk checker.
// Marks frame as failed when overlap already happened or the estimated
// time-to-collision (TTC) drops below configured critical levels.
class CollisionRiskChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  int total_frames_ = 0;
  int risky_frames_ = 0;
  int overlap_frames_ = 0;
  int imminent_frames_ = 0;
  double min_ttc_s_ = 1e9;
  double min_clearance_m_ = 1e9;
};

}  // namespace grading_mini
