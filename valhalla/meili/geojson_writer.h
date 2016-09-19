// -*- mode: c++ -*-
#ifndef MMP_GEOJSON_WRITER_H_
#define MMP_GEOJSON_WRITER_H_

#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/error/en.h>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/match_route.h>
#include <valhalla/meili/match_result.h>


namespace {

using namespace valhalla;

template <typename buffer_t>
void serialize_coordinate(rapidjson::Writer<buffer_t>& writer,
                          const midgard::PointLL& coord)
{
  writer.StartArray();
  // TODO lower precision
  writer.Double(coord.lng());
  writer.Double(coord.lat());
  writer.EndArray();
}


template <typename buffer_t>
void serialize_graphid(rapidjson::Writer<buffer_t>& writer,
                       const baldr::GraphId& graphid)
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
void serialize_routes(rapidjson::Writer<buffer_t>& writer,
                      const meili::MapMatcher& mapmatcher,
                      const meili::State& state)
{
  if (!state.routed()) {
    writer.Null();
    return;
  }

  const auto& mapmatching = mapmatcher.mapmatching();

  writer.StartArray();
  if (state.time() + 1 < mapmatching.size()) {
    for (const auto& next_state : mapmatching.states(state.time() + 1)) {
      auto label = state.RouteBegin(*next_state);
      if (label != state.RouteEnd()) {
        writer.StartObject();

        writer.String("next_state");
        writer.Uint(next_state->id());

        writer.String("edgeid");
        serialize_graphid(writer, label->edgeid);

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
          serialize_graphid(writer, label->edgeid);

          writer.String("nodeid");
          serialize_graphid(writer, label->nodeid);

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
void serialize_state(rapidjson::Writer<buffer_t>& writer,
                     const meili::State& state,
                     const meili::MapMatcher& mapmatcher)
{
  writer.StartObject();

  writer.String("id");
  writer.Uint(state.id());

  writer.String("time");
  writer.Uint(state.time());

  writer.String("distance");
  writer.Double(state.candidate().edges.front().score);

  //Note: technically a candidate can have correlated to more than one place in the graph
  //but the way its used in meili we only correlated it to one place so .front() is safe
  writer.String("coordinate");
  serialize_coordinate(writer, state.candidate().edges.front().projected);

  writer.String("routes");
  serialize_routes(writer, mapmatcher, state);

  writer.EndObject();
}


template <typename buffer_t>
void serialize_verbose(rapidjson::Writer<buffer_t>& writer,
                       const meili::MapMatcher& mapmatcher,
                       const std::vector<meili::MatchResult>& results)
{
  const auto& mapmatching = mapmatcher.mapmatching();

  writer.String("distances");
  writer.StartArray();
  for (const auto& result : results) {
    writer.Double(result.distance());
  }
  writer.EndArray();

  writer.String("graphids");
  writer.StartArray();
  for (const auto& result : results) {
    writer.Uint64(result.edgeid().id());
  }
  writer.EndArray();

  writer.String("states");
  writer.StartArray();
  for (const auto& result : results) {
    writer.StartArray();
    if (result.HasState()) {
      const auto& state = mapmatching.state(result.stateid());
      for (const auto& other_state : mapmatching.states(state.time())) {
        serialize_state(writer, *other_state, mapmatcher);
      }
    }
    writer.EndArray();
  }
  writer.EndArray();
}


}


namespace valhalla {
namespace meili {

template <typename buffer_t>
class GeoJSONWriter
{
 public:
  virtual ~GeoJSONWriter();

  virtual void WriteGeometry(rapidjson::Writer<buffer_t>& writer,
                             MapMatcher& matcher,
                             const std::vector<MatchResult>& results) const = 0;

  virtual void WriteGeometryCollection(rapidjson::Writer<buffer_t>& writer,
                                       MapMatcher& matcher,
                                       const std::vector<std::vector<MatchResult>>& result_lists) const;

  virtual void WriteProperties(rapidjson::Writer<buffer_t>& writer,
                               MapMatcher& matcher,
                               const std::vector<MatchResult>& results) const = 0;

  virtual void WriteFeature(rapidjson::Writer<buffer_t>& writer,
                            MapMatcher& matcher,
                            const std::vector<MatchResult>& results) const;

  virtual void WriteFeatureCollection(rapidjson::Writer<buffer_t>& writer,
                                      MapMatcher& matcher,
                                      const std::vector<std::vector<MatchResult>>& result_lists) const;
};


template <typename buffer_t>
GeoJSONWriter<buffer_t>::~GeoJSONWriter() {}


template <typename buffer_t>
void GeoJSONWriter<buffer_t>::WriteGeometryCollection(rapidjson::Writer<buffer_t>& writer,
                                                      MapMatcher& matcher,
                                                      const std::vector<std::vector<MatchResult>>& result_lists) const
{
  writer.StartObject();
  writer.String("type");
  writer.String("GeometryCollection");
  writer.String("geometries");
  writer.StartArray();
  for (const auto& results: result_lists) {
    WriteGeometry(writer, matcher, results);
  }
  writer.EndArray();
  writer.EndObject();
}


template <typename buffer_t>
void GeoJSONWriter<buffer_t>::WriteFeature(rapidjson::Writer<buffer_t>& writer,
                                           MapMatcher& matcher,
                                           const std::vector<MatchResult>& results) const
{
  writer.StartObject();

  writer.String("type");
  writer.String("Feature");

  writer.String("geometry");
  WriteGeometry(writer, matcher, results);

  writer.String("properties");
  WriteProperties(writer, matcher, results);

  writer.EndObject();
}


template <typename buffer_t>
void GeoJSONWriter<buffer_t>::WriteFeatureCollection(rapidjson::Writer<buffer_t>& writer,
                                                     MapMatcher& matcher,
                                                     const std::vector<std::vector<MatchResult>>& result_lists) const
{
  writer.StartObject();
  writer.String("type");
  writer.String("FeatureCollection");
  writer.String("features");
  writer.StartArray();
  for (const auto& results: result_lists) {
    WriteFeature(writer, matcher, results);
  }
  writer.EndArray();
  writer.EndObject();
}


template <typename buffer_t>
class GeoJSONRouteWriter: public GeoJSONWriter<buffer_t>
{
 public:
  GeoJSONRouteWriter(bool verbose=false);

  void WriteGeometry(rapidjson::Writer<buffer_t>& writer,
                     MapMatcher& matcher,
                     const std::vector<MatchResult>& results) const override;

  void WriteProperties(rapidjson::Writer<buffer_t>& writer,
                       MapMatcher& matcher,
                       const std::vector<MatchResult>& results) const override;

 private:
  bool verbose_;
};


template <typename buffer_t>
GeoJSONRouteWriter<buffer_t>::GeoJSONRouteWriter(bool verbose)
    :verbose_(verbose) {}


template <typename buffer_t>
void WriteRoute(rapidjson::Writer<buffer_t>& writer,
                MapMatcher& matcher,
                const std::vector<EdgeSegment>& route)
{
  // A state indicating if array is open (i.e. called
  // writer.StartArray() already but not writer.EndArray() yet)
  bool open = false;

  for (auto segment = route.cbegin(), prev_segment = route.cend();
       segment != route.cend(); segment++) {
    // Dummy segments should have been filtered out, however we still
    // do a check here
    if (!segment->edgeid.Is_Valid()) {
      continue;
    }

    const auto& shape = segment->Shape(matcher.graphreader());
    if (!shape.empty()) {
      const auto adjoined = prev_segment != route.cend() && prev_segment->Adjoined(matcher.graphreader(), *segment);
      // If current segment and previous segment adjoin, skip the
      // first coordinate since it's been written already
      if (adjoined) {
        for (auto vertex = std::next(shape.begin()); vertex != shape.end(); vertex++) {
          serialize_coordinate(writer, *vertex);
        }
      } else {
        // If current segment and previous segment don't adjoin, close
        // current array and start a new array
        if (open) {
          writer.EndArray();
          open = false;
        }
        writer.StartArray();
        open = true;
        for (auto vertex = shape.begin(); vertex != shape.end(); vertex++) {
          serialize_coordinate(writer, *vertex);
        }
      }
    }
    prev_segment = segment;
  }
  if (open) {
    writer.EndArray();
    open = false;
  }
}


template <typename buffer_t>
void GeoJSONRouteWriter<buffer_t>::WriteGeometry(rapidjson::Writer<buffer_t>& writer,
                                                 MapMatcher& mapmatcher,
                                                 const std::vector<MatchResult>& results) const
{
  writer.StartObject();

  writer.String("type");
  writer.String("MultiLineString");

  writer.String("coordinates");
  writer.StartArray();
  const auto& segments = ConstructRoute(mapmatcher.mapmatching(),
                                        results.cbegin(),
                                        results.cend());
  WriteRoute(writer, mapmatcher, segments);
  writer.EndArray();

  writer.EndObject();
}


template <typename buffer_t>
void GeoJSONRouteWriter<buffer_t>::WriteProperties(rapidjson::Writer<buffer_t>& writer,
                                                   MapMatcher& matcher,
                                                   const std::vector<MatchResult>& results) const
{
  writer.StartObject();
  writer.String("matched_coordinates");
  writer.StartArray();
  for (const auto& result : results) {
    if (result.edgeid().Is_Valid()) {
      serialize_coordinate(writer, result.lnglat());
    } else {
      writer.Null();
    }
  }
  writer.EndArray();
  if (verbose_) {
    serialize_verbose(writer, matcher, results);
  }
  writer.EndObject();
}


template <typename buffer_t>
class GeoJSONMatchedPointsWriter: public GeoJSONWriter<buffer_t>
{
 public:
  GeoJSONMatchedPointsWriter(bool verbose=false);

  void WriteGeometry(rapidjson::Writer<buffer_t>& writer,
                     MapMatcher& matcher,
                     const std::vector<MatchResult>& results) const override;

  void WriteProperties(rapidjson::Writer<buffer_t>& writer,
                       MapMatcher& matcher,
                       const std::vector<MatchResult>& results) const override;

 private:
  bool verbose_;
};


template <typename buffer_t>
GeoJSONMatchedPointsWriter<buffer_t>::GeoJSONMatchedPointsWriter(bool verbose)
    :verbose_(verbose) {}


template <typename buffer_t>
void GeoJSONMatchedPointsWriter<buffer_t>::WriteGeometry(rapidjson::Writer<buffer_t>& writer,
                                                         MapMatcher& matcher,
                                                         const std::vector<MatchResult>& results) const
{
  writer.StartObject();

  writer.String("type");
  writer.String("MultiPoint");

  writer.String("coordinates");
  writer.StartArray();
  for (const auto& result : results) {
    serialize_coordinate(writer, result.lnglat());
  }
  writer.EndArray();

  writer.EndObject();
}


template <typename buffer_t>
void GeoJSONMatchedPointsWriter<buffer_t>::WriteProperties(rapidjson::Writer<buffer_t>& writer,
                                                           MapMatcher& matcher,
                                                           const std::vector<MatchResult>& results) const
{
  writer.StartObject();
  if (verbose_) {
    serialize_verbose(writer, matcher, results);
  }
  writer.EndObject();
}

}
}
#endif // MMP_GEOJSON_WRITER_H_
