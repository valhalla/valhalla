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
constexpr float kDefaultSigmaZ = 4.07;
constexpr float kDefaultBeta = 5;
constexpr float kDefaultSquaredSearchRadius = 40 * 40;  // 40 meters
constexpr float kMaxSearchRadius = 200;                // 200 meters
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
void serialize_geometry(const std::vector<const State*>& path,
                        const MapMatching&mm,
                        Writer<T>& writer)
{
  writer.StartObject();
  writer.String("type");
  writer.String("MultiPoint");
  writer.String("coordinates");
  writer.StartArray();
  for (Time t = 0; t < path.size(); t++) {
    const auto& vertex = path[t]? path[t]->candidate().pathlocation().vertex() : mm.measurement(t).lnglat();
    serialize_coordinate(vertex, writer);
  }
  writer.EndArray();
  writer.EndObject();
}


template <typename T>
void serialize_labels(const State& state,
                      const MapMatching&mm,
                      Writer<T>& writer)
{
  if (!mm.has_labelset(state.id())) {
    writer.StartArray();
    writer.EndArray();
    return;
  }

  const auto& labelset = mm.labelset(state.id());
  writer.StartArray();
  if (state.time() + 1 < mm.size()) {
    for (const auto& next_state_ptr : mm.states(state.time() + 1)) {
      auto idx = mm.label_idx(state.id(), next_state_ptr->id());
      if (idx != kInvalidLabelIndex) {
        const auto& label = labelset.label(idx);
        writer.StartObject();

        writer.String("state");
        writer.Uint(next_state_ptr->id());

        writer.String("edge_id");
        writer.Uint(label.edgeid.id());

        writer.String("route_distance");
        writer.Double(label.cost);

        writer.EndObject();
      }
    }
  }
  writer.EndArray();
}


template <typename T>
void serialize_state(const State& state,
                     const MapMatching&mm,
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
void serialize_properties(const std::vector<const State*>& path,
                          const MapMatching&mm,
                          Writer<T>& writer)
{
  writer.StartObject();

  writer.String("states");
  writer.StartArray();
  for (Time t = 0; t < path.size(); t++) {
    writer.StartArray();
    for (const auto& state_ptr : mm.states(t)) {
      serialize_state(*state_ptr, mm, writer);
    }
    writer.EndArray();
  }
  writer.EndArray();
  writer.EndObject();
}


template <typename T>
void serialize_path(const std::vector<const State*>& path,
                    const MapMatching& mm,
                    Writer<T>& writer)
{
  writer.StartObject();

  writer.String("type");
  writer.String("Feature");

  writer.String("geometry");
  serialize_geometry(path, mm, writer);

  writer.String("properties");
  serialize_properties(path, mm, writer);

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


//TODO: throw this in the header to make it testable?
class mm_worker_t {
 public:
  mm_worker_t(const boost::property_tree::ptree& config):
      config(config),
      reader(config.get_child("mjolnir.hierarchy")),
      grid(reader, 0.25/1000, 0.25/1000),
      mode_costing{nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
        nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
        nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
        CreatePedestrianCost(*config.get_child_optional("costing_options.pedestrian"))} {
  }

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

      float sigma_z = kDefaultSigmaZ;
      if (json.HasMember("sigma_z")) {
        const auto& v = json["sigma_z"];
        if (v.IsNumber()) {
          sigma_z = static_cast<float>(v.GetDouble());
        }
      }

      float beta = kDefaultBeta;
      if (json.HasMember("beta")) {
        const auto& v = json["beta"];
        if (v.IsNumber()) {
          beta = static_cast<float>(v.GetDouble());
        }
      }

      LOG_INFO("Using sigma_z: " + std::to_string(sigma_z));
      LOG_INFO("Using beta: " + std::to_string(beta));

      // Match
      MapMatching mm(sigma_z, beta, reader, mode_costing, static_cast<sif::TravelMode>(3));

      float sq_search_radius = kDefaultSquaredSearchRadius;
      if (json.HasMember("radius")) {
        const auto& v = json["radius"];
        if (v.IsNumber()) {
          auto radius = std::max(0.f, std::min(static_cast<float>(v.GetDouble()), kMaxSearchRadius));
          sq_search_radius = radius * radius;
        }
      }

      LOG_INFO("Using search radius: " + std::to_string(std::sqrt(sq_search_radius)));

      auto path = OfflineMatch(mm, grid, measurements, sq_search_radius);
      assert(path.size() == measurements.size());

      // Serialize path
      StringBuffer sb;
      Writer<StringBuffer> writer(sb);
      serialize_path(path, mm, writer);

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
  }

 protected:
  boost::property_tree::ptree config;
  baldr::GraphReader reader;
  const CandidateGridQuery grid;
  std::shared_ptr<sif::DynamicCost> mode_costing[4];
};


void run_service(const boost::property_tree::ptree& config) {
  std::string proxy_endpoint("ipc:///tmp/mm_secret_proxy.your_pid");
  std::string server_endpoint("tcp://*:8001");
  std::string result_endpoint("ipc:///tmp/mm_result_proxy.your_pid");

  zmq::context_t context;

  http_server_t server(context, server_endpoint, proxy_endpoint + ".upstream", result_endpoint);
  std::thread server_thread = std::thread(std::bind(&http_server_t::serve, server));
  server_thread.detach();

  proxy_t proxy(context, proxy_endpoint + ".upstream", proxy_endpoint + ".downstream");
  std::thread proxy_thread(std::bind(&proxy_t::forward, proxy));
  proxy_thread.detach();

  //listen for requests
  mm_worker_t mm_worker(config);
  auto work = std::bind(&mm_worker_t::work, std::ref(mm_worker), std::placeholders::_1, std::placeholders::_2);
  auto cleanup = std::bind(&mm_worker_t::cleanup, std::ref(mm_worker));
  prime_server::worker_t worker(context, proxy_endpoint + ".downstream", "ipc://NO_ENDPOINT", result_endpoint, work, cleanup);
  worker.work();

  //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}


int main(int argc, char *argv[])
{
  boost::property_tree::ptree config;
  boost::property_tree::read_json("conf/valhalla.json", config);
  run_service(config);
  return 0;
}
