#pragma once

#include "src/grading/metric_base.h"

namespace grading_mini {

// Frame-level collision risk checker.
// Marks frame as failed when overlap already happened or the estimated
// time-to-collision (TTC) drops below configured critical levels.
// Irrelevant NPCs (off planned path, lateral pass-by, behind ego) are filtered.
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
  double warn_ttc_s_ = 3.0;
  double critical_ttc_s_ = 1.5;
  double near_clearance_m_ = 0.5;
  bool require_forward_corridor_ = true;
  double max_longitudinal_m_ = 40.0;
  double max_lateral_m_ = 1.2;
  bool require_approach_heading_ = true;
  double min_closing_speed_mps_ = 0.5;
  double min_forward_cos_ = 0.9396926207859083;  // cos(20 deg)
  bool require_planned_path_ = true;
  double path_lateral_tol_m_ = 1.0;
  double path_npc_width_ratio_ = 0.25;

  int total_frames_ = 0;
  int risky_frames_ = 0;
  int overlap_frames_ = 0;
  int imminent_frames_ = 0;
  double min_ttc_s_ = 1e9;
  double min_clearance_m_ = 1e9;
};

}  // namespace grading_mini
