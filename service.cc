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

#include "costings.h"
#include "map_matching.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include "rapidjson/stringbuffer.h"
#include <rapidjson/error/en.h>

using namespace rapidjson;
using namespace prime_server;
using namespace valhalla;


class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


void jsonify(Document& document, const char* text)
{
  document.Parse(text);

  if (document.HasParseError()) {
    std::string message(GetParseError_En(document.GetParseError()));
    throw SequenceParseError("Unable to parse JSON body: " + message);
  }
}


inline bool
is_geojson_geometry(const Value& geometry)
{
  return geometry.IsObject() && geometry.HasMember("coordinates");
}


std::vector<Measurement>
read_geojson_geometry(const Value& geometry)
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
  for (SizeType i = 0; i < coordinates.Size(); i++) {
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


// Strictly speak a GeoJSON feature must have "id" and "properties",
// but in our case they are optional

// {"type": "Feature", "geometry": GEOMETRY, "properties": {"times": [], "radius": []}}
inline bool
is_geojson_feature(const Value& object)
{
  return object.IsObject()
      && object.HasMember("type")
      && std::string(object["type"].GetString()) == "Feature"
      && object.HasMember("geometry");
}


std::vector<Measurement>
read_geojson_feature(const Value& feature)
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


std::vector<Measurement> read_geojson(const Value& object)
{
  if (is_geojson_feature(object)) {
    return read_geojson_feature(object);
  } else if (is_geojson_geometry(object)) {
    return read_geojson_geometry(object);
  } else {
    throw SequenceParseError("Invalid GeoJSON object: expect either Feature or Geometry");
  }
}


template <typename T>
void serialize_coordinate(const midgard::PointLL& coord, Writer<T>& writer)
{
  writer.StartArray();
  writer.Double(coord.lng());
  writer.Double(coord.lat());
  writer.EndArray();
}


template <typename T>
void serialize_geometry_matched_points(const std::vector<MatchResult>& results,
                                       Writer<T>& writer)
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


template <typename T>
void serialize_geometry_route(const std::vector<MatchResult>& results,
                              const MapMatching& mm,
                              Writer<T>& writer)
{
  writer.StartObject();

  writer.String("type");
  writer.String("MultiLineString");

  writer.String("coordinates");
  writer.StartArray();
  const auto& route = ConstructRoute(results.cbegin(), results.cend());
  bool open = false;
  for (auto segment = route.cbegin(), prev_segment = route.cend();
       segment != route.cend(); segment++) {
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


template <typename T>
void serialize_labels(const State& state,
                      const MapMatching& mm,
                      Writer<T>& writer)
{
  if (!state.routed()) {
    writer.StartArray();
    writer.EndArray();
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

        writer.String("edge_id");
        writer.Uint(label->edgeid.id());

        writer.String("route_distance");
        writer.Double(state.route_distance(*next_state));

        writer.String("route");
        writer.StartArray();
        for (auto label = state.RouteBegin(*next_state);
             label != state.RouteEnd();
             label++) {
          writer.StartObject();

          writer.String("edge_id");
          if (label->edgeid.Is_Valid()) {
            writer.Uint(label->edgeid.id());
          } else {
            writer.Null();
          }

          writer.String("node_id");
          if (label->nodeid.Is_Valid()) {
            writer.Uint(label->nodeid.id());
          } else {
            writer.Null();
          }

          writer.String("source");
          writer.Double(label->source);

          writer.String("target");
          writer.Double(label->target);

          writer.String("cost");
          writer.Double(label->cost);

          writer.EndObject();
        }
        writer.EndArray();

        writer.EndObject();
      }
    }
  }
  writer.EndArray();
}


template <typename T>
void serialize_state(const State& state,
                     const MapMatching& mm,
                     Writer<T>& writer)
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

  writer.String("labels");
  serialize_labels(state, mm, writer);

  writer.EndObject();
}


template <typename T>
void serialize_properties(const std::vector<MatchResult>& results,
                          const MapMatching& mm,
                          Writer<T>& writer)
{
  writer.StartObject();

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

  writer.EndObject();
}


template <typename T>
void serialize_results_as_feature(const std::vector<MatchResult>& results,
                                  const MapMatching& mm,
                                  Writer<T>& writer, bool route = false)
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


namespace {

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
}


const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};


worker_t::result_t jsonify_error(const std::string& message,
                                 http_request_t::info_t& info,
                                 unsigned status_code = 400)
{
  worker_t::result_t result{false};
  // {status: 400, message: "blah"}
  StringBuffer sb;
  Writer<StringBuffer> writer(sb);
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


inline float local_tile_size(const GraphReader& reader)
{
  const auto& tile_hierarchy = reader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  return tiles.TileSize();
}


boost::property_tree::ptree&
update_mm_config(boost::property_tree::ptree& config,
                 const http_request_t& request)
{
  const auto& query = request.query;
  const auto& customizables = config.get_child("customizable");
  for (const auto& item : customizables) {
    const auto& name = item.second.get_value<std::string>();
    const auto it = query.find(name);
    const auto& values = it->second;
    if (it != query.end() && !values.empty()) {
      if (name == "mode") {
        if (!values.back().empty()) {
          config.put<std::string>("mode", values.back());
        }
      } else if (name == "route" || name == "geometry") {
        if (values.back() == "false") {
          config.put<bool>(it->first, false);
        } else {
          config.put<bool>(it->first, true);
        }
      } else {
        if (!values.back().empty()) {
          try {
            // Possibly throw std::invalid_argument or std::out_of_range
            config.put<float>(it->first, std::stof(values.back()));
          } catch (const std::invalid_argument& ex) {
            throw std::invalid_argument("Invalid argument: unable to parse " + it->first + " to float");
          } catch (const std::out_of_range& ex) {
            throw std::out_of_range("Invalid argument: " + it->first + " is out of float range");
          }
        }
      }
    }
  }
  return config;
}


//TODO: throw this in the header to make it testable?
class mm_worker_t {
 public:
  mm_worker_t(const boost::property_tree::ptree& config):
      config(config),
      reader(config.get_child("mjolnir.hierarchy")),
      grid(reader,
           local_tile_size(reader)/config.get<size_t>("grid.size"),
           local_tile_size(reader)/config.get<size_t>("grid.size")),
      mode_costing(),
      factory_(),
      max_grid_cache_size_(config.get<size_t>("grid.cache_size")) {

    // Register edge/node costing methods
    factory_.Register("auto", sif::CreateAutoCost);
    factory_.Register("bicycle", sif::CreateBicycleCost);
    factory_.Register("pedestrian", sif::CreatePedestrianCost);
    factory_.Register("multimodal", mm::CreateUniversalCost);
  }

  sif::TravelMode init_and_find_costing(const std::string& mode_name)
  {
    sif::TravelMode travelmode;
    bool found = false;

    for (const auto& name : {"auto", "bicycle", "pedestrian", "multimodal"}) {
      const auto costing = factory_.Create(name, config.get_child(std::string("costing_options.") + name));
      mode_costing[static_cast<size_t>(costing->travelmode())] = costing;
      if (name == mode_name) {
        found = true;
        travelmode = costing->travelmode();
      }
    }
    if (!found) {
      throw std::runtime_error("Invalid mode " + mode_name);
    }

    return travelmode;
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
      Document json;

      // Parse sequence
      try {
        jsonify(json, request.body.c_str());
        measurements = read_geojson(json);
      } catch (const SequenceParseError& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Get a copy of the default config
      auto mm_config = config.get_child("mm");

      // Get mode name
      std::string mode_name;
      {
        const auto it = request.query.find("mode");
        if (it != request.query.end()) {
          if (!it->second.empty()) {
            mode_name = it->second.back();
          }
        }
        if (mode_name.empty()) {
          mode_name = mm_config.get<std::string>("mode");
        }
      }

      // Mode-specific config overwrites defaults
      {
        const auto mode_config = mm_config.get_child_optional(mode_name);
        if (mode_config) {
          for (const auto& child : *mode_config) {
            mm_config.put_child(child.first, child.second);
          }
        }
      }

      // Params in the request overwrite defaults
      try {
        update_mm_config(mm_config, request);
      } catch (const std::out_of_range& ex) {
        return jsonify_error(ex.what(), info);
      } catch (const std::invalid_argument& ex) {
        return jsonify_error(ex.what(), info);
      }

      float max_allowed_search_radius = mm_config.get<float>("max_search_radius"),
                        search_radius = mm_config.get<float>("search_radius");
      if (max_allowed_search_radius < search_radius) {
        std::string message = "Got search radius "
                              + std::to_string(search_radius)
                              + ", but we expect it not to exceed "
                              + std::to_string(max_allowed_search_radius);
        return jsonify_error(message, info);
      }

      sif::TravelMode travelmode;
      try {
        travelmode = init_and_find_costing(mode_name);
      } catch (const std::runtime_error& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Match
      MapMatching mm(reader, mode_costing, travelmode, mm_config);
      auto match_results = OfflineMatch(mm, grid, measurements, search_radius * search_radius);
      assert(match_results.size() == measurements.size());

      // Serialize results
      StringBuffer sb;
      Writer<StringBuffer> writer(sb);
      bool route = mm_config.get<bool>("route"),
        geometry = mm_config.get<bool>("geometry");
      if (geometry) {
        if (route) {
          serialize_geometry_route(match_results, mm, writer);
        } else {
          serialize_geometry_matched_points(match_results, writer);
        }
      } else {
        serialize_results_as_feature(match_results, mm, writer, route);
      }

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

  void cleanup() {
    if(reader.OverCommitted()) {
      reader.Clear();
    }
    if (grid.size() > max_grid_cache_size_) {
      grid.Clear();
    }
  }

 protected:
  boost::property_tree::ptree config;
  baldr::GraphReader reader;
  CandidateGridQuery grid;
  sif::cost_ptr_t mode_costing[32];
  sif::CostFactory<sif::DynamicCost> factory_;
  size_t max_grid_cache_size_;
};


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


int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cerr << "usage: service CONFIG" << std::endl;
    return 1;
  }

  std::string filename(argv[1]);
  boost::property_tree::ptree config;
  boost::property_tree::read_json(filename, config);
  run_service(config);

  return 0;
}
