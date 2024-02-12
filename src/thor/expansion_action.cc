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
  for (const auto& prop : options.expansion_properties()) {
    exp_props.insert(static_cast<Options_ExpansionProperties>(prop));
  }

  // default the expansion geojson so its easy to add to as we go
  writer_wrapper_t writer(1024 * 1024);
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");
  writer.set_precision(6);

  // a lambda that the path algorithm can call to add stuff to the dom
  // route and isochrone produce different GeoJSON properties
  std::string algo = "";
  auto track_expansion = [&writer, &opp_edges, &gen_factor, &skip_opps, &exp_props,
                          &algo](baldr::GraphReader& reader, baldr::GraphId edgeid,
                                 baldr::GraphId prev_edgeid, const char* algorithm = nullptr,
                                 std::string status = nullptr, const float duration = 0.f,
                                 const uint32_t distance = 0, const float cost = 0.f) {
    algo = algorithm;

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

    writer.start_object(); // feature object
    writer("type", "Feature");

    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    // make the geom
    for (const auto& p : shape) {
      writer.start_array();
      writer(p.lng());
      writer(p.lat());
      writer.end_array();
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry

    writer.start_object("properties");
    // no properties asked for, don't collect any
    if (!exp_props.size()) {
      writer.end_object(); // properties
      writer.end_object(); // feature
      return;
    }

    // make the properties
    if (exp_props.count(Options_ExpansionProperties_duration)) {
      writer("duration", static_cast<uint64_t>(duration));
    }
    if (exp_props.count(Options_ExpansionProperties_distance)) {
      writer("distance", static_cast<uint64_t>(distance));
    }
    if (exp_props.count(Options_ExpansionProperties_cost)) {
      writer("cost", static_cast<uint64_t>(cost));
    }
    if (exp_props.count(Options_ExpansionProperties_edge_status))
      writer("edge_status", status);
    if (exp_props.count(Options_ExpansionProperties_edge_id))
      writer("edge_id", static_cast<uint64_t>(edgeid));
    if (exp_props.count(Options_ExpansionProperties_pred_edge_id))
      writer("pred_edge_id", static_cast<uint64_t>(prev_edgeid));

    writer.end_object(); // properties
    writer.end_object(); // feature
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
  for (auto* alg : std::vector<MatrixAlgorithm*>{&costmatrix_, &time_distance_matrix_,
                                                 &time_distance_bss_matrix_}) {
    alg->set_track_expansion(track_expansion);
  }
  isochrone_gen.SetInnerExpansionCallback(track_expansion);

  try {
    // track the expansion
    if (exp_action == Options::route) {
      route(request);
    } else if (exp_action == Options::isochrone) {
      isochrones(request);
    } else if (exp_action == Options::sources_to_targets) {
      matrix(request);
    }
  } catch (...) {
    // we swallow exceptions because we actually want to see what the heck the expansion did
    // anyway
  }

  // close the GeoJSON
  writer.end_array(); // features
  writer.start_object("properties");
  writer("algorithm", algo);
  writer.end_object();
  writer.end_object(); // object

  // tell all the algorithms to stop tracking the expansion
  for (auto* alg : std::vector<PathAlgorithm*>{&multi_modal_astar, &timedep_forward, &timedep_reverse,
                                               &bidir_astar, &bss_astar}) {
    alg->set_track_expansion(nullptr);
  }
  costmatrix_.set_track_expansion(nullptr);
  isochrone_gen.SetInnerExpansionCallback(nullptr);

  // serialize it
  return writer.get_buffer();
}

} // namespace thor
} // namespace valhalla
