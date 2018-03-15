#include "test.h"

#include <string>
#include <vector>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "sif/dynamiccost.h"
#include "loki/worker.h"
#include "midgard/logging.h"

using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {

  // Quick costing class derived for testing so that any changes to regular costing
  // won't change the outcome of the tests. Some of the logic for this class is just
  // copy pasted from AutoCost as it stands when this test was written.
  class SimpleCost final : public DynamicCost {
  public:

    SimpleCost(const boost::property_tree::ptree& pt)
        : DynamicCost (pt, TravelMode::kDrive){}

    ~SimpleCost() {}

    uint32_t access_mode() const {
      return kAutoAccess;
    }

    bool Allowed(const DirectedEdge* edge,
          const EdgeLabel& pred,
          const GraphTile*& tile,
          const GraphId& edgeid) const {
      if (!(edge->forwardaccess() & kAutoAccess) ||
          (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
          (pred.restrictions() & (1 << edge->localedgeidx())) ||
           edge->surface() == Surface::kImpassable ||
           IsUserAvoidEdge(edgeid) ||
          (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
        return false;
      }
      return true;
    }

    bool AllowedReverse(const DirectedEdge* edge,
          const EdgeLabel& pred,
          const DirectedEdge* opp_edge,
          const GraphTile*& tile,
          const GraphId& opp_edgeid) const {
      if (!(opp_edge->forwardaccess() & kAutoAccess) ||
           (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
           (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
            opp_edge->surface() == Surface::kImpassable ||
            IsUserAvoidEdge(opp_edgeid) ||
           (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
        return false;
      }
      return true;
    }

    bool Allowed(const NodeInfo* node) const {
      return (node->access() & kAutoAccess);
    }

    Cost EdgeCost(const DirectedEdge* edge) const {
      float sec = static_cast<float>(edge->length());
      return {sec / 10.0f, sec};
    }

    Cost TransitionCost(const DirectedEdge* edge,
          const NodeInfo* node,
          const EdgeLabel& pred) const {
      return {5.0f, 5.0f};
    }

    Cost TransitionCostReverse(const uint32_t idx,
          const NodeInfo* node,
          const DirectedEdge* opp_edge,
          const DirectedEdge* opp_pred_edge) const {
      return {5.0f, 5.0f};
    }

    float AStarCostFactor() const {
      return 0.1f;
    }

    const EdgeFilter GetEdgeFilter() const {
      return [](const DirectedEdge* edge) {
        if (edge->IsTransition() || edge->is_shortcut() ||
           !(edge->forwardaccess() & kAutoAccess))
          return 0.0f;
        else {
          return 1.0f;
        }
      };
    }

    const NodeFilter GetNodeFilter() const {
      return [](const NodeInfo* node){
        return !(node->access() & kAutoAccess);
      };
    }

  };

  cost_ptr_t CreateSimpleCost (const boost::property_tree::ptree& pt) {
    return std::make_shared<SimpleCost> (pt);
  }


  boost::property_tree::ptree json_to_pt(const std::string& json) {
    std::stringstream ss; ss << json;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

  rapidjson::Document to_document(const std::string& request) {
    rapidjson::Document d;
    auto& allocator = d.GetAllocator();
    d.Parse(request.c_str());
    if (d.HasParseError())
      throw valhalla::valhalla_exception_t{100};
    return d;
  }
  void adjust_scores(valhalla::valhalla_request_t& request) {
    for(auto* locations : {request.options.mutable_locations(), request.options.mutable_sources(), request.options.mutable_targets()}) {
      for(auto& location : *locations) {
        auto minScoreEdge = *std::min_element (location.path_edges().begin(), location.path_edges().end(),
          [](valhalla::odin::Location::PathEdge i, valhalla::odin::Location::PathEdge j)->bool {
            return i.score() < j.score();
          });

        for(auto& e : *location.mutable_path_edges()) {
          e.set_score(e.score() - minScoreEdge.score());
          if (e.score() > 86400)
            e.set_score(86400);
        }
      }
    }
  }

  const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["sources_to_targets"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0}
    },
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

  const auto test_request = R"({
    "sources":[
      {"lat":52.106337,"lon":5.101728},
      {"lat":52.111276,"lon":5.089717},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.094273,"lon":5.075254}
    ],
    "costing":"auto"
  })";

  const auto test_request_osrm = R"({
    "sources":[
      {"lat":52.106337,"lon":5.101728},
      {"lat":52.111276,"lon":5.089717},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.094273,"lon":5.075254}
    ],
    "costing":"auto"
  }&format=osrm)";

  std::vector<TimeDistance> cost_matrix_answers = {
      {29, 29},
      {1967, 1852},
      {2329, 2225},
      {4084, 3854},
      {1488, 1398},
      {1739, 1639},
      {2065, 1981},
      {3857, 3641},
      {2313, 2213},
      {686, 641},
      {0, 0},
      {2803, 2643},
      {5529, 5279},
      {3902, 3707},
      {4302, 4108},
      {1810, 1680}
  };

  std::vector<TimeDistance> timedist_matrix_answers = {
      {28, 28},
      {2027, 1837},
      {2402, 2211},
      {4164, 3839},
      {1518, 1397},
      {1809, 1639},
      {2062, 1951},
      {3946, 3641},
      {2312, 2111},
      {700, 640},
      {0,0},
      {2822, 2626},
      {5563, 5177},
      {3951, 3706},
      {4367, 4106},
      {1825, 1679}
  };
}

void test_matrix() {
  loki_worker_t loki_worker (config);

  valhalla::valhalla_request_t request(test_request, valhalla::odin::DirectionsOptions::sources_to_targets);
  loki_worker.matrix (request);
  adjust_scores(request);

  auto request_pt = json_to_pt (rapidjson::to_string(request.document));

  GraphReader reader (config.get_child("mjolnir"));

  cost_ptr_t costing = CreateSimpleCost(request_pt);

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results;
  results = cost_matrix.SourceToTarget(request.options.sources(), request.options.targets(), reader, &costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist != cost_matrix_answers[i].dist) {
      throw std::runtime_error("result " + std::to_string(i) + "'s distance is not close enough"
          " to expected value for CostMatrix. Expected: " + std::to_string(cost_matrix_answers[i].dist)
          + " Actual: " + std::to_string(results[i].dist));
    }
    if (results[i].time != cost_matrix_answers[i].time) {
      throw std::runtime_error("result " + std::to_string(i) + "'s time is not close enough"
          " to expected value for CostMatrix. Expected: " + std::to_string(cost_matrix_answers[i].time)
          + " Actual: " + std::to_string(results[i].time));
    }
  }

  TimeDistanceMatrix timedist_matrix;
  results = timedist_matrix.SourceToTarget(request.options.sources(), request.options.targets(), reader, &costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist != timedist_matrix_answers[i].dist) {
      throw std::runtime_error("result " + std::to_string(i) + "'s distance is not equal to"
          " the expected value for TimeDistMatrix. Expected: " + std::to_string(timedist_matrix_answers[i].dist)
          + " Actual: " + std::to_string(results[i].dist));
    }
    if (results[i].time != timedist_matrix_answers[i].time) {
      throw std::runtime_error("result " + std::to_string(i) + "'s time is not equal to"
          " the expected value for TimeDistMatrix. Expected: " + std::to_string(timedist_matrix_answers[i].time)
          + " Actual: " + std::to_string(results[i].time));
    }
  }

}

void test_matrix_osrm() {
  loki_worker_t loki_worker (config);

  valhalla::valhalla_request_t request(test_request_osrm, valhalla::odin::DirectionsOptions::sources_to_targets);

  loki_worker.matrix (request);
  adjust_scores(request);
  auto request_pt = json_to_pt (rapidjson::to_string(request.document));

  GraphReader reader (config.get_child("mjolnir"));

  cost_ptr_t costing = CreateSimpleCost(request_pt);

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results;
  results = cost_matrix.SourceToTarget(request.options.sources(), request.options.targets(), reader, &costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist != cost_matrix_answers[i].dist) {
      throw std::runtime_error("Something is wrong");
      throw std::runtime_error("result " + std::to_string(i) + "'s distance is not close enough"
          " to expected value for CostMatrix. Expected: " + std::to_string(cost_matrix_answers[i].dist)
          + " Actual: " + std::to_string(results[i].dist));
    }
    if (results[i].time != cost_matrix_answers[i].time) {
      throw std::runtime_error("result " + std::to_string(i) + "'s time is not close enough"
          " to expected value for CostMatrix. Expected: " + std::to_string(cost_matrix_answers[i].time)
          + " Actual: " + std::to_string(results[i].time));
    }
  }

  TimeDistanceMatrix timedist_matrix;
  results = timedist_matrix.SourceToTarget(request.options.sources(), request.options.targets(), reader, &costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist != timedist_matrix_answers[i].dist) {
      throw std::runtime_error("result " + std::to_string(i) + "'s distance is not equal to"
          " the expected value for TimeDistMatrix. Expected: " + std::to_string(timedist_matrix_answers[i].dist)
          + " Actual: " + std::to_string(results[i].dist));
    }
    if (results[i].time != timedist_matrix_answers[i].time) {
      throw std::runtime_error("result " + std::to_string(i) + "'s time is not equal to"
          " the expected value for TimeDistMatrix. Expected: " + std::to_string(timedist_matrix_answers[i].time)
          + " Actual: " + std::to_string(results[i].time));
    }
  }

}

int main(int argc, char* argv[]) {
  test::suite suite ("matrix");
  logging::Configure({{"type", ""}}); //silence logs

  suite.test(TEST_CASE(test_matrix));
  //suite.test(TEST_CASE(test_matrix_osrm));

  return suite.tear_down();
}
