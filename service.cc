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


class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


std::vector<Measurement> parse_sequence(const char* text)
{
  Document document;
  document.Parse(text);

  if (document.HasParseError()) {
    auto message = GetParseError_En(document.GetParseError());
    throw SequenceParseError(message);
  }

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


Document serialize_path(const std::vector<const CandidateWrapper<Candidate>*>& path,
                        const std::vector<Measurement>&  measurements)
{
  Document document(kObjectType);
  auto& allocator = document.GetAllocator();

  Value matched_coordinates(kArrayType);
  matched_coordinates.Reserve(path.size(), allocator);

  for (decltype(path.size()) i = 0; i < path.size(); i++) {
    Value matched_coordinate(kArrayType);
    matched_coordinate.Reserve(2, allocator);
    if (path[i]) {
      const auto& vertex = path[i]->candidate().pathlocation().vertex();
      matched_coordinate.PushBack(vertex.lng(), allocator);
      matched_coordinate.PushBack(vertex.lat(), allocator);
    } else {
      const auto& lnglat = measurements[i].lnglat();
      matched_coordinate.PushBack(lnglat.lng(), allocator);
      matched_coordinate.PushBack(lnglat.lat(), allocator);
    }
    matched_coordinates.PushBack(matched_coordinate, allocator);
  }

  Value geometry(kObjectType);
  geometry.AddMember("type", "MultiPoint", allocator);
  geometry.AddMember("coordinates", matched_coordinates, allocator);

  // TODO add edge id associated with each matched point
  Value properties(kObjectType);

  document.AddMember("type", "Feature", allocator);
  document.AddMember("geometry", geometry, allocator);
  document.AddMember("properties", properties, allocator);

  return document;
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
      response.from_info(info);
      result.messages.emplace_back(response.to_string());
      return result;
    }

    if (request.method == method_t::POST) {
      std::vector<Measurement> measurements;
      // Parse sequence
      try {
        measurements = parse_sequence(request.body.c_str());
      } catch (const SequenceParseError& ex) {
        worker_t::result_t result{false};
        auto response = RESPONSE_400;
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }

      // Match
      // TODO read params from config or request
      MapMatching mm(4.07, 5, reader, mode_costing, static_cast<sif::TravelMode>(3));
      auto path = OfflineMatch(mm, grid, measurements, 1600);
      assert(path.size() == measurements.size());

      // Serialize path
      StringBuffer sb;
      Writer<StringBuffer> writer(sb);
      const auto& document = serialize_path(path, measurements);
      document.Accept(writer);

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
