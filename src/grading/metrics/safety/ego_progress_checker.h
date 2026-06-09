#pragma once

#include "proto/grading/metric_input.pb.h"
#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class EgoProgressChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  bool ComputeFrameScore(const proto::RoutePose2D& start,
                         const proto::RoutePose2D& end, double ego_x,
                         double ego_y, double* signed_progress_m,
                         double* along_ratio, double* frame_score,
                         bool* reached_end_line) const;

  double progress_exponent_ = 2.0;
  double pass_threshold_ = 0.95;

  bool has_sdc_route_ = false;
  bool warned_no_route_ = false;
  bool route_invalid_ = false;
  proto::SdcRouteContext sdc_route_;

  int total_frames_ = 0;
  int evaluated_frames_ = 0;
  int skipped_frames_ = 0;
  double max_frame_score_ = 0.0;
  double final_frame_score_ = 0.0;
  double max_signed_progress_m_ = -1e18;
};

}  // namespace grading_mini
