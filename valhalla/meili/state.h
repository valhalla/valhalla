// -*- mode: c++ -*-
#ifndef MMP_STATE_H_
#define MMP_STATE_H_

#include <unordered_map>
#include <vector>

#include <valhalla/baldr/pathlocation.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/meili/routing.h>
#include <valhalla/meili/stateid.h>
#include <valhalla/proto/tripcommon.pb.h>

namespace valhalla {
namespace meili {

class State {
public:
  State(const StateId& stateid, const baldr::PathLocation& candidate)
      : stateid_(stateid), candidate_(candidate), labelset_(nullptr), label_idx_() {
  }

  const StateId& stateid() const {
    return stateid_;
  }

  const baldr::PathLocation& candidate() const {
    return candidate_;
  }

  bool routed() const {
    return labelset_ != nullptr;
  }

  void SetRoute(const std::vector<StateId>& stateids,
                const std::unordered_map<uint16_t, uint32_t>& results,
                labelset_ptr_t labelset) const {
    if (!labelset) {
      throw std::runtime_error("expect valid labelset but got nullptr");
    }

    // Cache results
    label_idx_.clear();
    uint16_t dest = 1; // dest at 0 is remained for the origin
    for (const auto& stateid : stateids) {
      const auto it = results.find(dest);
      if (it != results.end()) {
        label_idx_[stateid] = it->second;
      }
      dest++;
    }

    labelset_ = labelset;
  }

  const Label* last_label(const State& state) const {
    const auto it = label_idx_.find(state.stateid());
    if (it != label_idx_.end()) {
      return &labelset_->label(it->second);
    }
    return nullptr;
  }

  RoutePathIterator RouteBegin(const State& state) const {
    const auto it = label_idx_.find(state.stateid());
    if (it != label_idx_.end()) {
      return RoutePathIterator(labelset_.get(), it->second);
    }
    return RoutePathIterator(labelset_.get());
  }

  RoutePathIterator RouteEnd() const {
    return RoutePathIterator(labelset_.get());
  }

private:
  StateId stateid_;

  baldr::PathLocation candidate_;

  mutable std::shared_ptr<LabelSet> labelset_;

  mutable std::unordered_map<StateId, uint32_t> label_idx_;
};

class StateContainer {
private:
  using Column = std::vector<State>;

public:
  StateContainer() : measurements_(), leave_times_(), columns_() {
  }

  void Clear() {
    measurements_.clear();
    leave_times_.clear();
    columns_.clear();
  }

  const State& state(const StateId& stateid) const {
    return columns_[stateid.time()][stateid.id()];
  }

  const Measurement& measurement(const StateId::Time& time) const {
    return measurements_[time];
  }

  double leave_time(const StateId::Time& time) const {
    return leave_times_[time];
  }

  void SetMeasurementLeaveTime(const StateId::Time& time, double leave_time) {
    leave_times_[time] = leave_time;
  }

  const Column& column(const StateId::Time& time) const {
    return columns_[time];
  }

  StateId::Time size() const {
    return static_cast<StateId::Time>(columns_.size());
  }

  std::string geojson(const StateId& s) {
    return geojson(state(s));
  }

  std::string geojson(const State& s) {
    std::stringstream ss;
    ss << std::setprecision(7) << std::fixed
       << R"({"type":"Feature","geometry":{"type":"Point","coordinates":[)";
    ss << s.candidate().edges[0].projected.lng() << ',' << s.candidate().edges[0].projected.lat()
       << "]}";
    ss << ',' << R"("properties":{"time":)" << s.stateid().time() << R"(,"id":)" << s.stateid().id()
       << R"(,"edge":")" << s.candidate().edges[0].id << "\"}}";
    return ss.str();
  }

  StateId NewStateId() const {
    return columns_.empty() ? StateId() : StateId(columns_.size() - 1, columns_.back().size());
  }

  StateId::Time AppendMeasurement(const Measurement& measurement) {
    const auto time = measurements_.size();

    measurements_.push_back(measurement);
    leave_times_.push_back(measurement.epoch_time());
    columns_.emplace_back();

    return time;
  }

  template <typename candidate_t> StateId AppendCandidate(candidate_t candidate) {
    const auto& stateid = NewStateId();
    AppendState(State(stateid, candidate));
    return stateid;
  }

  void AppendState(const State& state) {
    if (columns_.empty()) {
      throw std::runtime_error("add measurement first");
    }
    const auto expected_time = columns_.size() - 1;
    const auto expected_id = columns_.back().size();
    if (state.stateid() != StateId(expected_time, expected_id)) {
      throw std::runtime_error("state's stateid should be " + std::to_string(expected_time) + "/" +
                               std::to_string(expected_id) + " but got " +
                               std::to_string(state.stateid().time()) + "/" +
                               std::to_string(state.stateid().id()));
    }

    columns_.back().push_back(state);
  }

private:
  std::vector<Measurement> measurements_;

  std::vector<double> leave_times_;

  std::vector<Column> columns_;
};

} // namespace meili
} // namespace valhalla
#endif // MMP_STATE_H_
