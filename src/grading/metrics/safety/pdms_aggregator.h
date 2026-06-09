#pragma once

#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class PdmsAggregator : public MetricBase {
 public:
  PdmsAggregator();

  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  double weight_ep_ = 5.0;
  double weight_ttc_ = 5.0;
  double weight_c_ = 2.0;
  double weight_speed_ = 0.5;
  double pass_threshold_ = 0.95;
};

}  // namespace grading_mini
