#pragma once

#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"
#include "proto/grading/metrics/example_metric.pb.h"

namespace grading_mini {

class SpeedChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  double max_speed_ = 33.3;
  double min_speed_ = 2.0;
  double max_acceleration_ = 3.5;
  int overspeed_violations_ = 0;
  int too_slow_violations_ = 0;
  int excessive_accel_violations_ = 0;
  int total_frames_ = 0;
};

}  // namespace grading_mini
