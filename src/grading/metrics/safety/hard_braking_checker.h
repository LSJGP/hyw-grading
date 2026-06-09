#pragma once

#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class HardBrakingChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  double min_decel_threshold_mps2_ = 4.0;
  double min_speed_mps_ = 1.0;
  int violation_frames_ = 0;
  int total_frames_ = 0;
};

}  // namespace grading_mini
