#include "src/grading/metrics/safety/pdms_aggregator.h"

#include <sstream>

#include "google/protobuf/any.pb.h"
#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"

namespace grading_mini {
namespace {

constexpr const char* kNc = "regulatory_collision_checker";
constexpr const char* kDac = "drivable_area_checker";
constexpr const char* kSl = "solid_line_crossing_checker";
constexpr const char* kEp = "ego_progress_checker";
constexpr const char* kTtc = "collision_risk_checker";
constexpr const char* kC = "hard_braking_checker";
constexpr const char* kSpeed = "speed_checker";

bool FrameBoolScore(const MetricFrameOutput& output, bool default_if_missing) {
  if (output.value_one_of_case() == MetricFrameOutput::kBoolValue) {
    return output.bool_value();
  }
  return default_if_missing;
}

bool HasEgoProgressEval(const MetricFrameOutput& output) {
  proto::EgoProgressCheckerCustomInfo info;
  return output.custom_info().UnpackTo(&info);
}

double FrameEpScore(const MetricFrameOutput& output) {
  if (!HasEgoProgressEval(output)) {
    return 0.0;
  }
  if (output.value_one_of_case() == MetricFrameOutput::kDoubleValue) {
    return output.double_value();
  }
  proto::EgoProgressCheckerCustomInfo info;
  if (output.custom_info().UnpackTo(&info)) {
    return info.combined_frame_score();
  }
  return 0.0;
}

void AddSubScore(MetricSummary* summary, const std::string& name, double score) {
  auto* sub = summary->add_subscores();
  sub->set_name(name);
  sub->set_score(score);
}

const MetricFrameOutput* FrameAt(const Payload<MetricFrameOutput>& payload,
                                 size_t index) {
  const auto& data = payload.data();
  if (index >= data.size()) {
    return nullptr;
  }
  return &data[index];
}

double LastEpScore(const Payload<MetricFrameOutput>& ep_payload) {
  const auto& data = ep_payload.data();
  for (int i = static_cast<int>(data.size()) - 1; i >= 0; --i) {
    if (HasEgoProgressEval(data[static_cast<size_t>(i)])) {
      return FrameEpScore(data[static_cast<size_t>(i)]);
    }
  }
  return 0.0;
}

}  // namespace

REGISTER_METRIC(PdmsAggregator, "pdms_aggregator");

PdmsAggregator::PdmsAggregator() {
  deps_ = {kNc, kDac, kSl, kEp, kTtc, kC, kSpeed};
}

absl::Status PdmsAggregator::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::PdmsAggregatorConfig*>(config)) {
      if (typed->weight_ep() > 0.0) {
        weight_ep_ = typed->weight_ep();
      }
      if (typed->weight_ttc() > 0.0) {
        weight_ttc_ = typed->weight_ttc();
      }
      if (typed->weight_c() > 0.0) {
        weight_c_ = typed->weight_c();
      }
      if (typed->weight_speed() > 0.0) {
        weight_speed_ = typed->weight_speed();
      }
      if (typed->pass_threshold() > 0.0) {
        pass_threshold_ = typed->pass_threshold();
      }
    }
  }
  SPDLOG_INFO(
      "PdmsAggregator init: weights EP={:.1f} TTC={:.1f} C={:.1f} Speed={:.1f} "
      "pass_threshold={:.2f}",
      weight_ep_, weight_ttc_, weight_c_, weight_speed_, pass_threshold_);
  return absl::OkStatus();
}

absl::Status PdmsAggregator::CalculateOneFrame(
    const MetricFrameInput& /*input*/,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  output->set_bool_value(true);
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> PdmsAggregator::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  const auto& nc_payload = DependencyResult(kNc);
  const auto& dac_payload = DependencyResult(kDac);
  const auto& sl_payload = DependencyResult(kSl);
  const auto& ep_payload = DependencyResult(kEp);
  const auto& ttc_payload = DependencyResult(kTtc);
  const auto& c_payload = DependencyResult(kC);
  const auto& speed_payload = DependencyResult(kSpeed);

  const size_t frame_count = std::max({
      nc_payload.data().size(), dac_payload.data().size(),
      sl_payload.data().size(), ep_payload.data().size(),
      ttc_payload.data().size(), c_payload.data().size(),
      speed_payload.data().size()});

  double nc_run = 1.0;
  double dac_run = 1.0;
  double sl_run = 1.0;
  double ttc_sum = 0.0;
  int ttc_count = 0;
  double c_sum = 0.0;
  int c_count = 0;
  double speed_sum = 0.0;
  int speed_count = 0;

  for (size_t i = 0; i < frame_count; ++i) {
    const auto* nc = FrameAt(nc_payload, i);
    const auto* dac = FrameAt(dac_payload, i);
    const auto* sl = FrameAt(sl_payload, i);
    const auto* ttc = FrameAt(ttc_payload, i);
    const auto* c = FrameAt(c_payload, i);
    const auto* speed = FrameAt(speed_payload, i);

    const double nc_f = nc ? (FrameBoolScore(*nc, true) ? 1.0 : 0.0) : 0.0;
    const double dac_f = dac ? (FrameBoolScore(*dac, true) ? 1.0 : 0.0) : 0.0;
    const double sl_f = sl ? (FrameBoolScore(*sl, true) ? 1.0 : 0.0) : 0.0;

    nc_run = std::min(nc_run, nc_f);
    dac_run = std::min(dac_run, dac_f);
    sl_run = std::min(sl_run, sl_f);

    if (ttc) {
      ttc_sum += FrameBoolScore(*ttc, true) ? 1.0 : 0.0;
      ttc_count++;
    }

    if (c) {
      c_sum += FrameBoolScore(*c, true) ? 1.0 : 0.0;
      c_count++;
    }

    if (speed) {
      speed_sum += FrameBoolScore(*speed, true) ? 1.0 : 0.0;
      speed_count++;
    }
  }

  const double ep_final = LastEpScore(ep_payload);
  const double ttc_avg = ttc_count > 0 ? ttc_sum / ttc_count : 0.0;
  const double c_avg = c_count > 0 ? c_sum / c_count : 0.0;
  const double speed_avg = speed_count > 0 ? speed_sum / speed_count : 0.0;

  const double weight_sum =
      weight_ep_ + weight_ttc_ + weight_c_ + weight_speed_;
  const double weighted_avg =
      weight_sum > 0.0
          ? (weight_ep_ * ep_final + weight_ttc_ * ttc_avg + weight_c_ * c_avg +
             weight_speed_ * speed_avg) /
                weight_sum
          : 0.0;

  const double penalties = nc_run * dac_run * sl_run;
  const double pdms = penalties * weighted_avg;

  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_score(pdms);
  summary.set_passed(pdms >= pass_threshold_);

  AddSubScore(&summary, "NC", nc_run);
  AddSubScore(&summary, "DAC", dac_run);
  AddSubScore(&summary, "SL", sl_run);
  AddSubScore(&summary, "EP", ep_final);
  AddSubScore(&summary, "TTC", ttc_avg);
  AddSubScore(&summary, "C", c_avg);
  AddSubScore(&summary, "Speed", speed_avg);

  std::ostringstream oss;
  oss << "pdms=" << pdms << " penalties=" << penalties
      << " weighted_avg=" << weighted_avg << " frames=" << frame_count
      << " NC=" << nc_run << " DAC=" << dac_run << " SL=" << sl_run
      << " EP=" << ep_final << "(final_frame)"
      << " TTC=" << ttc_avg << " C=" << c_avg << " Speed=" << speed_avg;
  summary.set_detail(oss.str());

  return summary;
}

}  // namespace grading_mini
