#include <string>
#include <vector>
#include <thread>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>

#include "costings.h"
#include "map_matching.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include "rapidjson/stringbuffer.h"
#include <rapidjson/error/en.h>

using namespace rapidjson;
using namespace prime_server;
using namespace valhalla;


namespace {
constexpr float kSearchRadius = 40;      // meters
constexpr float kMaxSearchRadius = 200;  // meters
constexpr size_t kMaxGridCacheSize = 64; // quantity
constexpr size_t kGridSize = 500;        // quantity
}


class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


void jsonify(Document& document, const char* text)
{
  document.Parse(text);

  if (document.HasParseError()) {
    auto message = GetParseError_En(document.GetParseError());
    throw SequenceParseError(message);
  }
}


std::vector<Measurement> read_sequence(const Document& document)
{
  // Parse coordinates
  if (!document.HasMember("coordinates")) {
    throw SequenceParseError("coordinates not found");
  }

  const auto& coordinates = document["coordinates"];
  if (!coordinates.IsArray()) {
    throw SequenceParseError("coordindates is not an array of coordinates");
  }

  std::vector<Measurement> measurements;
  for (SizeType i = 0; i < coordinates.Size(); i++) {
    const auto& coordinate = coordinates[i];
    if (!coordinate.IsArray()
        || coordinate.Size() != 2
        || !coordinate[0].IsNumber()
        || !coordinate[1].IsNumber()) {
      throw SequenceParseError("coordindate at " + std::to_string(i) + " is not a valid coordinate (a array of two numbers)");
    }
    auto lng = coordinate[0].GetDouble(),
         lat = coordinate[1].GetDouble();
    measurements.emplace_back(midgard::PointLL(lng, lat));
  }

  // TODO parse timestamps
  if (document.HasMember("timestamps")) {
  }

  return measurements;
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
void serialize_geometry(const std::vector<MatchResult>& results,
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
void serialize_labels(const State& state,
                      const MapMatching&mm,
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
      if (label != state.RouteEnd(*next_state)) {
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
             label != state.RouteEnd(*next_state);
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
  serialize_coordinate(state.candidate().pathlocation().vertex(), writer);

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
    writer.Double(result.graphid().id());
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
void serialize_results(const std::vector<MatchResult>& results,
                       const MapMatching& mm,
                       Writer<T>& writer)
{
  writer.StartObject();

  writer.String("type");
  writer.String("Feature");

  writer.String("geometry");
  serialize_geometry(results, writer);

  writer.String("properties");
  serialize_properties(results, mm, writer);

  writer.EndObject();
}


const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

const http_response_t RESPONSE_400(400, "Bad Request", "Malformed HTTP request", {CORS});
const http_response_t RESPONSE_405(405, "Method Not Allowed", "The request method is inappropriate for the URL", {CORS});
const http_response_t RESPONSE_413(413, "Request Entity Too Large", "The HTTP request was too large", {CORS});
const http_response_t RESPONSE_501(501, "Not Implemented", "The HTTP request method is not supported", {CORS});
const http_response_t RESPONSE_505(505, "HTTP Version Not Supported", "The HTTP request version is not supported", {CORS});


inline float local_tile_size(const GraphReader& reader)
{
  const auto& tile_hierarchy = reader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  return tiles.TileSize();
}


namespace {
const char* const kCustomizableOptions[] = {
  "beta", "sigma_z", "route_distance_factor",
  "breakage_distance", "interpolation_distance",
  "search_radius"
};
}


boost::property_tree::ptree&
update_mm_config(boost::property_tree::ptree& pt,
                 const http_request_t& request)
{
  const auto& query = request.query;
  for (const auto& name : kCustomizableOptions) {
    const auto it = query.find(name);
    if (it != query.end()) {
      const auto& value_list = it->second;
      if (!value_list.empty()) {
        // Possibly throw std::invalid_argument or std::out_of_range
        pt.put<float>(it->first, std::stof(value_list.back()));
      }
    }
  }
  return pt;
}


boost::property_tree::ptree&
update_mm_config(boost::property_tree::ptree& pt,
                 const Document& document)
{
  if (document.HasMember("properties")) {
    return pt;
  }
  const auto& properties = document["properties"];
  for (const auto& name : kCustomizableOptions) {
    if (properties.HasMember(name)) {
      auto value = properties[name].GetDouble();
      if (std::numeric_limits<float>::max() <= value
          && value <= std::numeric_limits<float>::max()) {
        pt.put<float>(name, static_cast<float>(value));
      } else {
        throw std::out_of_range("out of float range");
      }
    }
  }
  return pt;
}


//TODO: throw this in the header to make it testable?
class mm_worker_t {
 public:
  mm_worker_t(const boost::property_tree::ptree& config):
      config(config),
      reader(config.get_child("mjolnir.hierarchy")),
      grid(reader,
           local_tile_size(reader)/config.get<size_t>("grid.size", kGridSize),
           local_tile_size(reader)/config.get<size_t>("grid.size", kGridSize)),
      mode_costing{nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
        nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
        nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
        CreateUniversalCost(*config.get_child_optional("costing_options.pedestrian"))},
      max_grid_cache_size_(config.get<size_t>("grid.cache_size", kMaxGridCacheSize)) {}

  worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
    auto& info = *static_cast<http_request_t::info_t*>(request_info);
    LOG_INFO("Got Map Matching Request " + std::to_string(info.id));

    http_request_t request;

    // Parse HTTP
    try {
      request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
    } catch(const std::runtime_error& ex) {
      worker_t::result_t result{false};
      auto response = RESPONSE_400;
      response.body = ex.what();
      response.from_info(info);
      result.messages.emplace_back(response.to_string());
      return result;
    }

    if (request.method == method_t::POST) {
      std::vector<Measurement> measurements;
      // Parse sequence
      Document json;
      try {
        jsonify(json, request.body.c_str());
        measurements = read_sequence(json);
      } catch (const SequenceParseError& ex) {
        worker_t::result_t result{false};
        auto response = RESPONSE_400;
        response.body = ex.what();
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }

      // Read default config
      auto mm_config = config.get_child("mm");

      float max_allowed_search_radius = mm_config.get<float>("max_search_radius", kMaxSearchRadius);

      // Update customizable configs
      update_mm_config(mm_config, request);
      update_mm_config(mm_config, json);

      float search_radius = mm_config.get<float>("search_radius", kSearchRadius);
      if (search_radius > max_allowed_search_radius) {
        std::string message = "Got search radius "
                              + std::to_string(search_radius)
                              + ", but we expect it not to exceed "
                              + std::to_string(max_allowed_search_radius);
        worker_t::result_t result{false};
        auto response = RESPONSE_400;
        response.body = message;
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }

      // Match
      MapMatching mm(reader, mode_costing, static_cast<sif::TravelMode>(3), mm_config);
      auto match_results = OfflineMatch(mm, grid, measurements, search_radius * search_radius);
      assert(match_results.size() == measurements.size());

      // Serialize results
      StringBuffer sb;
      Writer<StringBuffer> writer(sb);
      serialize_results(match_results, mm, writer);

      worker_t::result_t result{false};
      http_response_t response(200, "OK", sb.GetString(), headers_t{CORS, JS_MIME});
      response.from_info(info);
      result.messages.emplace_back(response.to_string());
      return result;
    } else {
      // Method not support
      worker_t::result_t result{false};
      auto response = RESPONSE_405;
      response.from_info(info);
      result.messages.emplace_back(response.to_string());
      return result;
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
  std::shared_ptr<sif::DynamicCost> mode_costing[4];
  size_t max_grid_cache_size_;
};


void run_service(const boost::property_tree::ptree& config) {
  std::string proxy_endpoint = config.get<std::string>("mm.service.proxy");
  // Run as a standalone service
  std::string server_endpoint = config.get<std::string>("mm.service.listen");
  std::string loopback_endpoint = config.get<std::string>("mm.service.loopback");

  zmq::context_t context;

  http_server_t server(context, server_endpoint, proxy_endpoint + ".upstream", loopback_endpoint);
  std::thread server_thread = std::thread(std::bind(&http_server_t::serve, server));
  server_thread.detach();

  proxy_t proxy(context, proxy_endpoint + ".upstream", proxy_endpoint + ".downstream");
  std::thread proxy_thread(std::bind(&proxy_t::forward, proxy));
  proxy_thread.detach();

  //listen for requests
  mm_worker_t mm_worker(config);
  auto work = std::bind(&mm_worker_t::work, std::ref(mm_worker), std::placeholders::_1, std::placeholders::_2);
  auto cleanup = std::bind(&mm_worker_t::cleanup, std::ref(mm_worker));
  prime_server::worker_t worker(context, proxy_endpoint + ".downstream", "ipc://NO_ENDPOINT", loopback_endpoint,
                                work, cleanup);
  worker.work();

  //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
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
