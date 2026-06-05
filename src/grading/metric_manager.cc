#include "src/grading/metric_manager.h"

#include <sstream>

#include "spdlog/spdlog.h"

#include "src/grading/macros.h"

namespace grading_mini {

namespace {

std::string FormatLogLine(int64_t frame_id, int64_t timestamp_us, double speed_mps,
                          bool collided, bool passed) {
  std::ostringstream oss;
  oss << "frame=" << frame_id << " t=" << (timestamp_us / 1e6) << "s"
      << " v=" << speed_mps << " coll=" << (collided ? "Y" : "n") << " "
      << (passed ? "PASS" : "FAIL");
  return oss.str();
}

}  // namespace

absl::Status MetricManager::AddMetric(const std::string& name,
                                      std::unique_ptr<MetricBase> metric) {
  if (metrics_.count(name)) {
    return absl::AlreadyExistsError("Metric [" + name + "] already exists");
  }
  metrics_.emplace(name, std::move(metric));
  payloads_.emplace(name, std::make_unique<Payload<MetricFrameOutput>>());
  return absl::OkStatus();
}

absl::Status MetricManager::BuildGraph() {
  std::vector<std::string> names;
  names.reserve(metrics_.size());
  for (const auto& [n, _] : metrics_) names.push_back(n);

  DAGScheduler scheduler(names);
  for (const auto& [name, metric] : metrics_) {
    for (const auto& dep : metric->dependencies()) {
      RETURN_IF_ERROR(scheduler.AddDependency(name, dep));
    }
  }

  auto plan_or = scheduler.GenerateUpdatePlan();
  RETURN_IF_ERROR(plan_or.status());
  plan_ = std::make_unique<DAGScheduler::UpdatePlan>(
      std::move(plan_or).value());

  for (const auto& name : names) {
    for (const auto& dep : metrics_[name]->dependencies()) {
      auto it = payloads_.find(dep);
      if (it == payloads_.end()) {
        return absl::NotFoundError("Payload for dep [" + dep + "] not found");
      }
      RETURN_IF_ERROR(
          metrics_[name]->SetDependencyPayload(dep, it->second.get()));
    }
  }

  SPDLOG_INFO("DAG built: {} metrics, {} levels", names.size(), plan_->size());
  return absl::OkStatus();
}

absl::Status MetricManager::RunOneFrame(const MetricFrameInput& input) {
  if (!plan_) return absl::FailedPreconditionError("Graph not built");

  FrameContext ctx;
  ctx.frame_id = input.frame_id();
  ctx.timestamp_us = input.timestamp_us();
  ctx.speed_mps = input.vehicle_state().speed();
  ctx.collided =
      input.has_collision_event() && input.collision_event().collided();
  frame_contexts_.push_back(ctx);

  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto& metric = metrics_[name];
      auto& payload = payloads_[name];
      MetricFrameOutput output;

      RETURN_IF_ERROR(
          metric->CalculateOneFrame(input, payload->data(), &output));

      output.set_frame_id(input.frame_id());
      output.set_timestamp_us(input.timestamp_us());
      payload->InsertData(output);
      payload->MaintainOnce();
    }
  }
  return absl::OkStatus();
}

std::vector<std::pair<std::string, bool>>
MetricManager::LastFrameVerdicts() const {
  std::vector<std::pair<std::string, bool>> out;
  if (!plan_) return out;
  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto it = payloads_.find(name);
      if (it == payloads_.end() || it->second->data().empty()) continue;
      out.emplace_back(name, it->second->data().back().bool_value());
    }
  }
  return out;
}

absl::StatusOr<proto::GradingReport> MetricManager::GenerateReport() {
  if (!plan_) return absl::FailedPreconditionError("Graph not built");

  proto::GradingReport report;
  report.set_overall_passed(true);

  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto& metric = metrics_[name];
      auto& payload = payloads_[name];

      auto summary_or = metric->SummarizeResult(payload->data());
      if (!summary_or.ok()) {
        SPDLOG_WARN("Summarize [{}] failed: {}", name,
                    std::string(summary_or.status().message()));
        continue;
      }

      auto summary = std::move(summary_or).value();
      if (summary.metric_name().empty()) summary.set_metric_name(name);
      report.add_summaries()->CopyFrom(summary);

      if (!summary.passed()) report.set_overall_passed(false);
    }
  }
  return report;
}

absl::StatusOr<std::vector<proto::MetricDetailReport>>
MetricManager::GenerateMetricDetailReports() {
  if (!plan_) return absl::FailedPreconditionError("Graph not built");

  std::unordered_map<int64_t, const FrameContext*> ctx_by_frame;
  ctx_by_frame.reserve(frame_contexts_.size());
  for (const auto& ctx : frame_contexts_) {
    ctx_by_frame[ctx.frame_id] = &ctx;
  }

  std::vector<proto::MetricDetailReport> reports;
  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto& metric = metrics_[name];
      auto& payload = payloads_[name];

      auto summary_or = metric->SummarizeResult(payload->data());
      if (!summary_or.ok()) {
        SPDLOG_WARN("Summarize [{}] failed: {}", name,
                    std::string(summary_or.status().message()));
        continue;
      }

      const auto summary = std::move(summary_or).value();
      proto::MetricDetailReport detail;
      detail.set_metric_name(summary.metric_name().empty() ? name
                                                           : summary.metric_name());
      detail.set_passed(summary.passed());
      detail.set_summary(summary.detail());

      for (const auto& output : payload->data()) {
        auto* frame = detail.add_frames();
        frame->set_frame_id(output.frame_id());
        frame->set_passed(output.bool_value());

        const FrameContext* ctx = nullptr;
        auto it = ctx_by_frame.find(output.frame_id());
        if (it != ctx_by_frame.end()) {
          ctx = it->second;
        }

        const int64_t timestamp_us =
            output.timestamp_us() != 0
                ? output.timestamp_us()
                : (ctx ? ctx->timestamp_us : 0);
        frame->set_timestamp_us(timestamp_us);
        frame->set_speed_mps(ctx ? ctx->speed_mps : 0.0);
        frame->set_collided(ctx ? ctx->collided : false);

        frame->set_log_line(FormatLogLine(
            output.frame_id(), timestamp_us, ctx ? ctx->speed_mps : 0.0,
            ctx ? ctx->collided : false, output.bool_value()));

        if (output.has_custom_info()) {
          frame->mutable_custom_info()->CopyFrom(output.custom_info());
        }
      }

      reports.push_back(std::move(detail));
    }
  }
  return reports;
}

}  // namespace grading_mini
