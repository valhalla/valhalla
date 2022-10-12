#include "thor/worker.h"

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"

using namespace rapidjson;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

// indices correspond to Options::ExpansionProperties enum
const std::string kPropPaths[5] = {"/features/0/properties/costs", "/features/0/properties/durations",
                                   "/features/0/properties/distances",
                                   "/features/0/properties/statuses",
                                   "/features/0/properties/edge_ids"};

std::string thor_worker_t::expansion(Api& request) {
  // time this whole method and save that statistic
  measure_scope_time(request);

  // get the request params
  auto options = request.options();
  auto exp_action = options.expansion_action();
  bool skip_opps = options.skip_opposites();
  std::unordered_set<baldr::GraphId> opp_edges;
  std::unordered_set<Options::ExpansionProperties> exp_props;

  // default generalization to ~ zoom level 15
  float gen_factor = options.has_generalize_case() ? options.generalize() : 10.f;

  // default the expansion geojson so its easy to add to as we go
  Document dom;
  dom.SetObject();
  // set algorithm to Dijkstra, will be overwritten by other algos
  SetValueByPointer(dom, "/type", "FeatureCollection");
  SetValueByPointer(dom, "/features/0/type", "Feature");
  SetValueByPointer(dom, "/features/0/geometry/type", "MultiLineString");
  SetValueByPointer(dom, "/features/0/geometry/coordinates", Value(kArrayType));
  SetValueByPointer(dom, "/features/0/properties", Value(kObjectType));
  for (const auto& prop : options.expansion_properties()) {
    rapidjson::Pointer(kPropPaths[prop]).Set(dom, Value(kArrayType));
    exp_props.insert(static_cast<Options_ExpansionProperties>(prop));
  }

  // a lambda that the path algorithm can call to add stuff to the dom
  // route and isochrone produce different GeoJSON properties
  auto track_expansion = [&dom, &opp_edges, &gen_factor, &skip_opps,
                          &exp_props](baldr::GraphReader& reader, baldr::GraphId edgeid,
                                      const char* algorithm = nullptr, const char* status = nullptr,
                                      const float duration = 0.f, const uint32_t distance = 0,
                                      const float cost = 0.f) {
    auto tile = reader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      LOG_ERROR("thor_worker_t::expansion error, tile no longer available" +
                std::to_string(edgeid.Tile_Base()));
      return;
    }
    const auto* edge = tile->directededge(edgeid);
    // unfortunately we have to call this before checking if we can skip
    // else the tile could change underneath us when we get the opposing
    auto shape = tile->edgeinfo(edge).shape();
    auto names = tile->edgeinfo(edge).GetNames();
    auto is_forward = edge->forward();

    // if requested, skip this edge in case its opposite edge has been added
    // before (i.e. lower cost) else add this edge's id to the lookup container
    if (skip_opps) {
      auto opp_edgeid = reader.GetOpposingEdgeId(edgeid, tile);
      if (opp_edgeid && opp_edges.count(opp_edgeid))
        return;
      opp_edges.insert(edgeid);
    }

    if (!edge->forward())
      std::reverse(shape.begin(), shape.end());
    Polyline2<PointLL>::Generalize(shape, gen_factor, {}, false);

    // make the geom
    auto& a = dom.GetAllocator();
    auto* coords = GetValueByPointer(dom, "/features/0/geometry/coordinates");
    coords->GetArray().PushBack(Value(kArrayType), a);
    auto& linestring = (*coords)[coords->Size() - 1];
    for (const auto& p : shape) {
      linestring.GetArray().PushBack(Value(kArrayType), a);
      auto point = linestring[linestring.Size() - 1].GetArray();
      point.PushBack(p.first, a);
      point.PushBack(p.second, a);
    }

    // no properties asked for, don't collect any
    if (!exp_props.size()) {
      return;
    }

    // make the properties
    SetValueByPointer(dom, "/properties/algorithm", algorithm);
    if (exp_props.count(Options_ExpansionProperties_durations)) {
      Pointer(kPropPaths[Options_ExpansionProperties_durations])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<uint64_t>(duration)), a);
    }
    if (exp_props.count(Options_ExpansionProperties_distances)) {
      Pointer(kPropPaths[Options_ExpansionProperties_distances])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(distance), a);
    }
    if (exp_props.count(Options_ExpansionProperties_costs)) {
      Pointer(kPropPaths[Options_ExpansionProperties_costs])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<uint64_t>(cost)), a);
    }
    if (exp_props.count(Options_ExpansionProperties_statuses))
      Pointer(kPropPaths[Options_ExpansionProperties_statuses])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetString(status, a), a);
    if (exp_props.count(Options_ExpansionProperties_edge_ids))
      Pointer(kPropPaths[Options_ExpansionProperties_edge_ids])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<uint64_t>(edgeid)), a);
  };

  // tell all the algorithms how to track expansion
  for (auto* alg : std::vector<PathAlgorithm*>{
           &multi_modal_astar,
           &timedep_forward,
           &timedep_reverse,
           &bidir_astar,
           &bss_astar,
       }) {
    alg->set_track_expansion(track_expansion);
  }
  isochrone_gen.SetInnerExpansionCallback(track_expansion);

  try {
    // track the expansion
    if (exp_action == Options::route) {
      route(request);
    } else if (exp_action == Options::isochrone) {
      isochrones(request);
    }
  } catch (...) {
    // we swallow exceptions because we actually want to see what the heck the expansion did
    // anyway
  }

  // tell all the algorithms to stop tracking the expansion
  for (auto* alg : std::vector<PathAlgorithm*>{&multi_modal_astar, &timedep_forward, &timedep_reverse,
                                               &bidir_astar, &bss_astar}) {
    alg->set_track_expansion(nullptr);
  }
  isochrone_gen.SetInnerExpansionCallback(nullptr);

  // serialize it
  return to_string(dom, 5);
}

} // namespace thor
} // namespace valhalla
