#include <string>
#include <vector>
#include <thread>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/sif/costfactory.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include "rapidjson/stringbuffer.h"
#include <rapidjson/error/en.h>

#include "mmp/universal_cost.h"
#include "mmp/map_matching.h"

using namespace prime_server;
using namespace valhalla;
using namespace mmp;

#define VERBOSE


namespace
{

constexpr size_t kHttpStatusCodeSize = 600;
const char* kHttpStatusCodes[kHttpStatusCodeSize];


inline const std::string
http_status_code(unsigned code)
{
  return code < kHttpStatusCodeSize? kHttpStatusCodes[code] : "";
}


// Credits: http://werkzeug.pocoo.org/
void init_http_status_codes()
{
  // 1xx
  kHttpStatusCodes[100] = "Continue";
  kHttpStatusCodes[101] = "Switching Protocols";
  kHttpStatusCodes[102] = "Processing";

  // 2xx
  kHttpStatusCodes[200] = "OK";
  kHttpStatusCodes[201] = "Created";
  kHttpStatusCodes[202] = "Accepted";
  kHttpStatusCodes[203] = "Non Authoritative Information";
  kHttpStatusCodes[204] = "No Content";
  kHttpStatusCodes[205] = "Reset Content";
  kHttpStatusCodes[206] = "Partial Content";
  kHttpStatusCodes[207] = "Multi Status";
  kHttpStatusCodes[226] = "IM Used";  // see RFC 322

  // 3xx
  kHttpStatusCodes[300] = "Multiple Choices";
  kHttpStatusCodes[301] = "Moved Permanently";
  kHttpStatusCodes[302] = "Found";
  kHttpStatusCodes[303] = "See Other";
  kHttpStatusCodes[304] = "Not Modified";
  kHttpStatusCodes[305] = "Use Proxy";
  kHttpStatusCodes[307] = "Temporary Redirect";

  // 4xx
  kHttpStatusCodes[400] = "Bad Request";
  kHttpStatusCodes[401] = "Unauthorized";
  kHttpStatusCodes[402] = "Payment Required";  // unuse
  kHttpStatusCodes[403] = "Forbidden";
  kHttpStatusCodes[404] = "Not Found";
  kHttpStatusCodes[405] = "Method Not Allowed";
  kHttpStatusCodes[406] = "Not Acceptable";
  kHttpStatusCodes[407] = "Proxy Authentication Required";
  kHttpStatusCodes[408] = "Request Timeout";
  kHttpStatusCodes[409] = "Conflict";
  kHttpStatusCodes[410] = "Gone";
  kHttpStatusCodes[411] = "Length Required";
  kHttpStatusCodes[412] = "Precondition Failed";
  kHttpStatusCodes[413] = "Request Entity Too Large";
  kHttpStatusCodes[414] = "Request URI Too Long";
  kHttpStatusCodes[415] = "Unsupported Media Type";
  kHttpStatusCodes[416] = "Requested Range Not Satisfiable";
  kHttpStatusCodes[417] = "Expectation Failed";
  kHttpStatusCodes[418] = "I\'m a teapot";  // see RFC 232
  kHttpStatusCodes[422] = "Unprocessable Entity";
  kHttpStatusCodes[423] = "Locked";
  kHttpStatusCodes[424] = "Failed Dependency";
  kHttpStatusCodes[426] = "Upgrade Required";
  kHttpStatusCodes[428] = "Precondition Required";  // see RFC 658
  kHttpStatusCodes[429] = "Too Many Requests";
  kHttpStatusCodes[431] = "Request Header Fields Too Large";
  kHttpStatusCodes[449] = "Retry With";  // proprietary MS extensio

  // 5xx
  kHttpStatusCodes[500] = "Internal Server Error";
  kHttpStatusCodes[501] = "Not Implemented";
  kHttpStatusCodes[502] = "Bad Gateway";
  kHttpStatusCodes[503] = "Service Unavailable";
  kHttpStatusCodes[504] = "Gateway Timeout";
  kHttpStatusCodes[505] = "HTTP Version Not Supported";
  kHttpStatusCodes[507] = "Insufficient Storage";
  kHttpStatusCodes[510] = "Not Extended";
}


const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};


class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


void jsonify(rapidjson::Document& document, const char* text)
{
  document.Parse(text);

  if (document.HasParseError()) {
    std::string message(GetParseError_En(document.GetParseError()));
    throw SequenceParseError("Unable to parse JSON body: " + message);
  }
}


inline bool
is_geojson_geometry(const rapidjson::Value& geometry)
{
  return geometry.IsObject() && geometry.HasMember("coordinates");
}


std::vector<Measurement>
read_geojson_geometry(const rapidjson::Value& geometry)
{
  if (!is_geojson_geometry(geometry)) {
    throw SequenceParseError("Invalid GeoJSON geometry");
  }

  // Parse coordinates
  if (!geometry.HasMember("coordinates")) {
    throw SequenceParseError("Invalid GeoJSON geometry: coordinates not found");
  }

  const auto& coordinates = geometry["coordinates"];
  if (!coordinates.IsArray()) {
    throw SequenceParseError("Invalid GeoJSON geometry: coordindates is not an array of coordinates");
  }

  std::vector<Measurement> measurements;
  for (rapidjson::SizeType i = 0; i < coordinates.Size(); i++) {
    const auto& coordinate = coordinates[i];
    if (!coordinate.IsArray()
        || coordinate.Size() != 2
        || !coordinate[0].IsNumber()
        || !coordinate[1].IsNumber()) {
      throw SequenceParseError("Invalid GeoJSON geometry: coordindate at "
                               + std::to_string(i)
                               + " is not a valid coordinate (a array of two numbers)");
    }
    auto lng = coordinate[0].GetDouble(),
         lat = coordinate[1].GetDouble();
    measurements.emplace_back(midgard::PointLL(lng, lat));
  }

  return measurements;
}


inline bool
is_geojson_feature(const rapidjson::Value& object)
{
  // Strictly speak a GeoJSON feature must have "id" and "properties",
  // but in our case they are optional. A full example is as folllows:
  // {"id": 1, "type": "Feature", "geometry": GEOMETRY, "properties": {"times": [], "radius": []}}

  // We follow Postel's Law: be liberal in what you accept
  return object.IsObject()
      // && object.HasMember("type")
      // && std::string(object["type"].GetString()) == "Feature"
      && object.HasMember("geometry");
}


std::vector<Measurement>
read_geojson_feature(const rapidjson::Value& feature)
{
  if (!is_geojson_feature(feature)) {
    throw SequenceParseError("Invalid GeoJSON feature");
  }

  auto measurements = read_geojson_geometry(feature["geometry"]);

  if (feature.HasMember("properties")) {
    // TODO add time and accuracy
  }

  return measurements;
}


std::vector<Measurement> read_geojson(const rapidjson::Value& object)
{
  if (is_geojson_feature(object)) {
    return read_geojson_feature(object);
  } else if (is_geojson_geometry(object)) {
    return read_geojson_geometry(object);
  } else {
    throw SequenceParseError("Invalid GeoJSON object: expect either Feature or Geometry");
  }
}


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
void serialize_geometry_matched_points(const std::vector<MatchResult>& results,
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
                          rapidjson::Writer<buffer_t>& writer)
{
  writer.StartObject();

  writer.String("matched_points");
  writer.StartArray();
  for (const auto& result : results) {
    serialize_coordinate(result.lnglat(), writer);
  }
  writer.EndArray();

#ifdef VERBOSE
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
#endif

  writer.EndObject();
}


template <typename buffer_t>
void serialize_results_as_feature(const std::vector<MatchResult>& results,
                                  const MapMatching& mm,
                                  rapidjson::Writer<buffer_t>& writer,
                                  bool route = false)
{
  writer.StartObject();

  writer.String("type");
  writer.String("Feature");

  writer.String("geometry");
  if (route) {
    serialize_geometry_route(results, mm, writer);
  } else {
    serialize_geometry_matched_points(results, writer);
  }

  writer.String("properties");
  serialize_properties(results, mm, writer);

  writer.EndObject();
}


template <typename buffer_t>
void serialize_config(MapMatcher* matcher,
                      rapidjson::Writer<buffer_t>& writer)
{
  // Property tree -> string
  std::stringstream ss;
  boost::property_tree::json_parser::write_json(ss, matcher->config());
  const auto& str = ss.str();

  // String -> JSON document
  rapidjson::Document document;
  document.Parse(str.c_str());

  // JSON document -> writer stream
  document.Accept(writer);
}


template <typename buffer_t>
void serialize_response(buffer_t& sb,
                        const std::vector<MatchResult>& results,
                        MapMatcher* matcher)
{
  rapidjson::Writer<buffer_t> writer(sb);
  bool route = matcher->config().get<bool>("route"),
    geometry = matcher->config().get<bool>("geometry");
  writer.StartObject();

  writer.String("status");
  writer.Uint(200);

  writer.String("message");
  // Need the reference otherwise the c string will be invalidated?
  // TODO possible to make a c string version of http_status_code?
  const auto& status = http_status_code(200);
  writer.String(status.c_str());

  writer.String("data");
  if (geometry) {
    if (route) {
      serialize_geometry_route(results, matcher->mapmatching(), writer);
    } else {
      serialize_geometry_matched_points(results, writer);
    }
  } else {
    serialize_results_as_feature(results, matcher->mapmatching(), writer, route);
  }

#ifdef VERBOSE
  writer.String("config");
  serialize_config(matcher, writer);
#endif

  writer.EndObject();
}


worker_t::result_t jsonify_error(const std::string& message,
                                 http_request_t::info_t& info,
                                 unsigned status_code = 400)
{
  worker_t::result_t result{false};
  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);

  writer.StartObject();

  writer.String("status");
  writer.Uint(status_code);

  writer.String("message");
  writer.String(message.c_str());

  writer.EndObject();

  http_response_t response(status_code, http_status_code(status_code), sb.GetString(), {JSON_MIME, CORS});
  response.from_info(info);
  result.messages.emplace_back(response.to_string());

  return result;
}


template <typename T>
inline std::unordered_set<T>
ptree_array_to_unordered_set(const boost::property_tree::ptree& ptree)
{
  std::unordered_set<T> result;
  for (const auto& item : ptree) {
    result.insert(item.second.get_value<T>());
  }
  return result;
}


//TODO: throw this in the header to make it testable?
class mm_worker_t {
 public:
  mm_worker_t(const boost::property_tree::ptree& config)
      : config_(config),
        matcher_factory_(config_),
        customizable_(ptree_array_to_unordered_set<std::string>(config_.get_child("mm.customizable"))) {}

  boost::property_tree::ptree&
  read_preferences_from_request(const http_request_t& request,
                                boost::property_tree::ptree& preferences)
  {
    if (customizable_.empty()) {
      return preferences;
    }
    const auto& query = request.query;
    for (const auto& pair : query) {
      const auto& name = pair.first;
      const auto& values = pair.second;
      if (customizable_.find(name) != customizable_.end()
          && !values.empty()) {
        // String
        if (name == "mode") {
          if (!values.back().empty()) {
            preferences.put<std::string>("mode", values.back());
          }
        }
        // Boolean
        else if (name == "route" || name == "geometry") {
          if (values.back() == "false") {
            preferences.put<bool>(name, false);
          } else {
            preferences.put<bool>(name, true);
          }
        }
        // Float
        else {
          if (!values.back().empty()) {
            try {
              // Possibly throw std::invalid_argument or std::out_of_range
              preferences.put<float>(name, std::stof(values.back()));
            } catch (const std::invalid_argument& ex) {
              throw std::invalid_argument("Invalid argument: unable to parse " + name + " to float");
            } catch (const std::out_of_range& ex) {
              throw std::out_of_range("Invalid argument: " + name + " is out of float range");
            }
          }
        }
      }
    }

    return preferences;
  }

  worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
    auto& info = *static_cast<http_request_t::info_t*>(request_info);
    LOG_INFO("Got Map Matching Request " + std::to_string(info.id));

    http_request_t request;

    // Parse HTTP
    try {
      request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
    } catch (const std::runtime_error& ex) {
      return jsonify_error(ex.what(), info);
    }

    if (request.method == method_t::POST) {
      std::vector<Measurement> measurements;
      rapidjson::Document json;

      // Parse sequence
      try {
        jsonify(json, request.body.c_str());
        measurements = read_geojson(json);
      } catch (const SequenceParseError& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Read preferences
      boost::property_tree::ptree preferences;
      try {
        read_preferences_from_request(request, preferences);
      } catch (const std::invalid_argument& ex) {
        return jsonify_error(ex.what(), info);
      } catch (const std::out_of_range& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Create a matcher
      MapMatcher* matcher;
      try {
        matcher = matcher_factory_.Create(preferences);
      } catch (const std::invalid_argument& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Match
      const auto& results = matcher->OfflineMatch(measurements);

      // Serialize results
      rapidjson::StringBuffer sb;
      serialize_response(sb, results, matcher);

      delete matcher;

      worker_t::result_t result{false};
      http_response_t response(200, http_status_code(200), sb.GetString(), headers_t{CORS, JS_MIME});
      response.from_info(info);
      result.messages.emplace_back(response.to_string());
      return result;
    } else {
      // Method not support
      return jsonify_error(http_status_code(405), info, 405);
    }
  }

  void cleanup()
  { matcher_factory_.ClearCacheIfPossible(); }

 protected:
  const boost::property_tree::ptree config_;
  MapMatcherFactory matcher_factory_;
  std::unordered_set<std::string> customizable_;
};


}


namespace mmp
{

void run_service(const boost::property_tree::ptree& config)
{
  std::string proxy_endpoint = config.get<std::string>("mm.service.proxy"),
     proxy_upstream_endpoint = proxy_endpoint + ".upstream",
   proxy_downstream_endpoint = proxy_endpoint + ".downstream",
           loopback_endpoint = config.get<std::string>("mm.service.loopback"),
             server_endpoint = config.get<std::string>("mm.service.listen");

  zmq::context_t context;

  init_http_status_codes();

  http_server_t server(context, server_endpoint, proxy_upstream_endpoint, loopback_endpoint);
  std::thread server_thread = std::thread(std::bind(&http_server_t::serve, server));
  server_thread.detach();

  proxy_t proxy(context, proxy_upstream_endpoint, proxy_downstream_endpoint);
  std::thread proxy_thread(std::bind(&proxy_t::forward, proxy));
  proxy_thread.detach();

  // Listen for requests
  mm_worker_t mm_worker(config);
  auto work = std::bind(&mm_worker_t::work, std::ref(mm_worker), std::placeholders::_1, std::placeholders::_2);
  auto cleanup = std::bind(&mm_worker_t::cleanup, std::ref(mm_worker));
  prime_server::worker_t worker(context,
                                proxy_downstream_endpoint, "ipc:///dev/null", loopback_endpoint,
                                work, cleanup);

  worker.work();

  //TODO should we listen for SIGINT and terminate gracefully/exit(0)?
}

}
