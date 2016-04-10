// -*- mode: c++ -*-

#ifndef MMP_GEOJSON_WRITER_H_
#define MMP_GEOJSON_WRITER_H_

#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/error/en.h>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>


namespace mmp {

using namespace valhalla;

template <typename buffer_t>
void serialize_coordinate(const midgard::PointLL& coord,
                          rapidjson::Writer<buffer_t>& writer)
{
  writer.StartArray();
  // TODO lower precision
  writer.Double(coord.lng());
  writer.Double(coord.lat());
  writer.EndArray();
}


template <typename buffer_t>
void serialize_graphid(const baldr::GraphId& graphid,
                       rapidjson::Writer<buffer_t>& writer)
{
  if (graphid.Is_Valid()) {
    writer.StartObject();

    writer.String("id");
    writer.Uint(graphid.id());

    writer.String("level");
    writer.Uint(graphid.level());

    writer.String("tileid");
    writer.Uint(graphid.tileid());

    writer.EndObject();
  } else {
    writer.Null();
  }
}


template <typename buffer_t>
void serialize_geometry_matched_coordinates(const std::vector<MatchResult>& results,
                                            rapidjson::Writer<buffer_t>& writer)
{
  writer.StartObject();

  writer.String("type");
  writer.String("MultiPoint");

  writer.String("coordinates");
  writer.StartArray();
  for (const auto& result : results) {
    serialize_coordinate(result.lnglat(), writer);
  }
  writer.EndArray();

  writer.EndObject();
}


template <typename buffer_t>
void serialize_geometry_route(const std::vector<MatchResult>& results,
                              const MapMatching& mm,
                              rapidjson::Writer<buffer_t>& writer)
{
  writer.StartObject();

  writer.String("type");
  writer.String("MultiLineString");

  writer.String("coordinates");
  writer.StartArray();
  const auto& route = ConstructRoute(mm.graphreader(), results.cbegin(), results.cend());
  bool open = false;
  for (auto segment = route.cbegin(), prev_segment = route.cend();
       segment != route.cend(); segment++) {
    assert(segment->edgeid.Is_Valid());
    const auto& shape = segment->Shape(mm.graphreader());
    if (!shape.empty()) {
      assert(shape.size() >= 2);
      if (prev_segment != route.cend()
          && prev_segment->Adjoined(mm.graphreader(), *segment)) {
        for (auto vertex = std::next(shape.begin()); vertex != shape.end(); vertex++) {
          serialize_coordinate(*vertex, writer);
        }
      } else {
        if (open) {
          writer.EndArray();
          open = false;
        }
        writer.StartArray();
        open = true;
        for (auto vertex = shape.begin(); vertex != shape.end(); vertex++) {
          serialize_coordinate(*vertex, writer);
        }
      }
    }
    prev_segment = segment;
  }
  if (open) {
    writer.EndArray();
    open = false;
  }
  writer.EndArray();

  writer.EndObject();
}


template <typename buffer_t>
void serialize_routes(const State& state,
                      const MapMatching& mm,
                      rapidjson::Writer<buffer_t>& writer)
{
  if (!state.routed()) {
    writer.Null();
    return;
  }

  writer.StartArray();
  if (state.time() + 1 < mm.size()) {
    for (const auto& next_state : mm.states(state.time() + 1)) {
      auto label = state.RouteBegin(*next_state);
      if (label != state.RouteEnd()) {
        writer.StartObject();

        writer.String("next_state");
        writer.Uint(next_state->id());

        writer.String("edgeid");
        serialize_graphid(label->edgeid, writer);

        const auto label = state.last_label(*next_state);
        writer.String("route_distance");
        writer.Double(label->cost);

        writer.String("route_turn_cost");
        writer.Double(label->turn_cost);

        writer.String("route");
        writer.StartArray();
        for (auto label = state.RouteBegin(*next_state);
             label != state.RouteEnd();
             label++) {
          writer.StartObject();

          writer.String("edgeid");
          serialize_graphid(label->edgeid, writer);

          writer.String("nodeid");
          serialize_graphid(label->nodeid, writer);

          writer.String("source");
          writer.Double(label->source);

          writer.String("target");
          writer.Double(label->target);

          writer.String("route_distance");
          writer.Double(label->cost);

          writer.String("turn_cost");
          writer.Double(label->turn_cost);

          writer.EndObject();
        }
        writer.EndArray();

        writer.EndObject();
      }
    }
  }
  writer.EndArray();
}


template <typename buffer_t>
void serialize_state(const State& state,
                     const MapMatching& mm,
                     rapidjson::Writer<buffer_t>& writer)
{
  writer.StartObject();

  writer.String("id");
  writer.Uint(state.id());

  writer.String("time");
  writer.Uint(state.time());

  writer.String("distance");
  writer.Double(state.candidate().distance());

  writer.String("coordinate");
  serialize_coordinate(state.candidate().vertex(), writer);

  writer.String("routes");
  serialize_routes(state, mm, writer);

  writer.EndObject();
}


template <typename buffer_t>
void serialize_properties(const std::vector<MatchResult>& results,
                          const MapMatching& mm,
                          rapidjson::Writer<buffer_t>& writer,
                          bool verbose)
{
  writer.StartObject();

  writer.String("matched_coordinates");
  writer.StartArray();
  for (const auto& result : results) {
    serialize_coordinate(result.lnglat(), writer);
  }
  writer.EndArray();

  if (verbose) {
    writer.String("distances");
    writer.StartArray();
    for (const auto& result : results) {
      writer.Double(result.distance());
    }
    writer.EndArray();

    writer.String("graphids");
    writer.StartArray();
    for (const auto& result : results) {
      writer.Uint64(result.graphid().id());
    }
    writer.EndArray();

    writer.String("states");
    writer.StartArray();
    for (const auto& result : results) {
      writer.StartArray();
      if (result.state()) {
        for (const auto state : mm.states(result.state()->time())) {
          serialize_state(*state, mm, writer);
        }
      }
      writer.EndArray();
    }
    writer.EndArray();
  }

  writer.EndObject();
}


template <typename buffer_t>
void serialize_results_as_feature(const std::vector<MatchResult>& results,
                                  const MapMatching& mm,
                                  rapidjson::Writer<buffer_t>& writer,
                                  bool route,
                                  bool verbose)
{
  writer.StartObject();

  writer.String("type");
  writer.String("Feature");

  writer.String("geometry");
  if (route) {
    serialize_geometry_route(results, mm, writer);
  } else {
    serialize_geometry_matched_coordinates(results, writer);
  }

  writer.String("properties");
  serialize_properties(results, mm, writer, verbose);

  writer.EndObject();
}

}


#endif // MMP_GEOJSON_WRITER_H_
