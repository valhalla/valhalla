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
void FillDom(Document& dom, bool is_route_expansion) {
  SetValueByPointer(dom, "/type", "FeatureCollection");
  SetValueByPointer(dom, "/features/0/type", "Feature");
  SetValueByPointer(dom, "/features/0/geometry/type", "MultiLineString");
  SetValueByPointer(dom, "/features/0/geometry/coordinates", Value(kArrayType));
  if (is_route_expansion) {
    SetValueByPointer(dom, "/properties/algorithm", "none");
    SetValueByPointer(dom, "/features/0/properties/edge_ids", Value(kArrayType));
    SetValueByPointer(dom, "/features/0/properties/statuses", Value(kArrayType));
  } else {
    SetValueByPointer(dom, "/features/0/properties/durations", Value(kArrayType));
    SetValueByPointer(dom, "/features/0/properties/distances", Value(kArrayType));
  }
}
} // namespace

namespace valhalla {
namespace thor {

std::string thor_worker_t::expansion(Api& request) {
  // time this whole method and save that statistic
  measure_scope_time(request, "thor_worker_t::expansion");

  bool is_route_expansion = request.options().expansion_type() ==
                            Options::ExpansionType::Options_ExpansionType_expand_route;

  // default generalization to ~ zoom level 15, minor impact anyways
  float gen_factor = request.options().has_generalize() ? request.options().generalize() : 21.4f;

  // default the expansion geojson so its easy to add to as we go
  Document dom;
  dom.SetObject();
  FillDom(dom, is_route_expansion);

  // a lambda that the path algorithm can call to add stuff to the dom
  // route and isochrone produce different GeoJSON properties
  auto track_expansion = [&dom, &is_route_expansion,
                          &gen_factor](baldr::GraphReader& reader, baldr::GraphId edgeid,
                                       const char* algorithm = nullptr, const char* status = nullptr,
                                       const float duration = 0, const float distance = 0) {
    // full shape might be overkill but meh, its trace
    auto tile = reader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      LOG_ERROR("thor_worker_t::expansion error, tile no longer available" +
                std::to_string(edgeid.Tile_Base()));
      return;
    }
    const auto* edge = tile->directededge(edgeid);
    auto shape = tile->edgeinfo(edge).shape();
    if (!edge->forward())
      std::reverse(shape.begin(), shape.end());
    Polyline2<PointLL>::Generalize(shape, gen_factor, {}, false);

    // make the geom
    auto& a = dom.GetAllocator();
    auto* coords = GetValueByPointer(dom, "/features/0/geometry/coordinates");

    // make the properties and coords
    if (is_route_expansion) {
      coords->GetArray().PushBack(Value(kArrayType), a);
      auto& linestring = (*coords)[coords->Size() - 1];
      for (const auto& p : shape) {
        linestring.GetArray().PushBack(Value(kArrayType), a);
        auto point = linestring[linestring.Size() - 1].GetArray();
        point.PushBack(p.first, a);
        point.PushBack(p.second, a);
      }
      SetValueByPointer(dom, "/properties/algorithm", algorithm);
      GetValueByPointer(dom, "/features/0/properties/edge_ids")
          ->GetArray()
          .PushBack(static_cast<uint64_t>(edgeid), a);
      GetValueByPointer(dom, "/features/0/properties/statuses")
          ->GetArray()
          .PushBack(Value{}.SetString(status, a), a);

    } else {
      coords->GetArray().PushBack(Value{}.SetString(encode<std::vector<PointLL>>(shape, 1e5), a), a);
      GetValueByPointer(dom, "/features/0/properties/durations")
          ->GetArray()
          .PushBack(Value{}.SetInt(static_cast<u_int64_t>(duration)), a);
      GetValueByPointer(dom, "/features/0/properties/distances")
          ->GetArray()
          .PushBack(Value{}.SetInt(distance), a);
    }
  };

  if (is_route_expansion) {
    // tell all the algorithms how to track expansion
    for (auto* alg : std::vector<PathAlgorithm*>{
             &multi_modal_astar,
             &timedep_forward,
             &timedep_reverse,
             &bidir_astar,
             &bss_astar,
         }) {
      alg->set_track_expansion(track_expansion);

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
    }
  } else {
    isochrone_gen.set_track_expansion(track_expansion);
    isochrones(request);
    isochrone_gen.set_track_expansion(nullptr);
  }

  // serialize it
  return to_string(dom, 5);
}

} // namespace thor
} // namespace valhalla