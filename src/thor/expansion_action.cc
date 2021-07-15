#include "thor/worker.h"

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"

using namespace rapidjson;
using namespace valhalla::midgard;

namespace {
// these need to represent array types
enum PropType {
  kEdgeId,
  kStatus,
  kDist,
  kTime,
  kCost,
  // insert new keys before this
  LAST
};

static std::unordered_map<PropType, const char*> kPropPaths =
    {{PropType::kEdgeId, "/features/0/properties/edge_ids"},
     {PropType::kStatus, "/features/0/properties/statuses"},
     {PropType::kDist, "/features/0/properties/distances"},
     {PropType::kTime, "/features/0/properties/durations"},
     {PropType::kCost, "/features/0/properties/costs"}};
} // namespace

namespace valhalla {
namespace thor {

std::string thor_worker_t::expansion(Api& request) {
  // time this whole method and save that statistic
  measure_scope_time(request, "thor_worker_t::expansion");

  // get the request params
  auto options = request.options();
  auto exp_action = options.expansion_action();
  bool skip_opps = options.skip_opposites();
  std::unordered_set<baldr::GraphId> opp_edges;

  // default generalization to ~ zoom level 15
  float gen_factor = options.has_generalize() ? options.generalize() : 21.4f;

  // default the expansion geojson so its easy to add to as we go
  Document dom;
  dom.SetObject();
  // set algorithm to Dijkstra, will be overwritten by other algos
  SetValueByPointer(dom, "/properties/algorithm", "unidirectional_dijkstra");
  SetValueByPointer(dom, "/type", "FeatureCollection");
  SetValueByPointer(dom, "/features/0/type", "Feature");
  SetValueByPointer(dom, "/features/0/geometry/type", "MultiLineString");
  SetValueByPointer(dom, "/features/0/geometry/coordinates", Value(kArrayType));
  for (int prop = PropType::kEdgeId; prop != LAST; prop++) {
    rapidjson::Pointer(kPropPaths[static_cast<PropType>(prop)]).Set(dom, Value(kArrayType));
  }

  // a lambda that the path algorithm can call to add stuff to the dom
  // route and isochrone produce different GeoJSON properties
  auto track_expansion = [&dom, &exp_action, &opp_edges, &gen_factor,
                          &skip_opps](baldr::GraphReader& reader, baldr::GraphId edgeid,
                                      const char* algorithm = nullptr, const char* status = nullptr,
                                      const float duration = 0, const u_int32_t distance = 0,
                                      const float cost = 0) {
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

    // make the properties
    if (algorithm) {
      SetValueByPointer(dom, "/properties/algorithm", algorithm);
    }
    if (duration)
      Pointer(kPropPaths[PropType::kTime])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<u_int64_t>(duration)), a);
    if (distance)
      Pointer(kPropPaths[PropType::kDist]).Get(dom)->GetArray().PushBack(Value{}.SetInt(distance), a);
    if (cost)
      Pointer(kPropPaths[PropType::kCost])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<u_int64_t>(cost)), a);
    if (status)
      Pointer(kPropPaths[PropType::kStatus])
          .Get(dom)
          ->GetArray()
          .PushBack(Value{}.SetString(status, a), a);
    // edgeid will be populated either way, but isochrones don't want it
    if (exp_action == Options_Action_route)
      Pointer(kPropPaths[PropType::kEdgeId])
          .Get(dom)
          ->GetArray()
          .PushBack(static_cast<uint64_t>(edgeid), a);
  };

  if (exp_action == Options::route) {
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

    // track the expansion
    try {
      route(request);
    } catch (...) {
      // we swallow exceptions because we actually want to see what the heck the expansion did
      // anyway
    }

    // tell all the algorithms to stop tracking the expansion
    for (auto* alg : std::vector<PathAlgorithm*>{
             &multi_modal_astar,
             &timedep_forward,
             &timedep_reverse,
             &bidir_astar,
             &bss_astar,
         }) {
      alg->set_track_expansion(nullptr);
    }
  } else {
    isochrone_gen.set_track_expansion(track_expansion);
    isochrones(request, true);
    isochrone_gen.set_track_expansion(nullptr);
  }

  // remove the arrays which weren't used
  for (int prop = PropType::kEdgeId; prop != LAST; prop++) {
    const auto& member = kPropPaths[static_cast<PropType>(prop)];
    if (!Pointer(member).Get(dom)->GetArray().Size()) {
      Pointer(member).Erase(dom);
    }
  }

  // serialize it
  return to_string(dom, 5);
}

} // namespace thor
} // namespace valhalla