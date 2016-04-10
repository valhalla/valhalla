#include <string>
#include <vector>
#include <thread>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>

#include <rapidjson/stringbuffer.h>

#include "mmp/measurement.h"
#include "mmp/universal_cost.h"
#include "mmp/map_matching.h"
#include "mmp/geojson_reader.h"
#include "mmp/geojson_writer.h"

using namespace prime_server;
using namespace valhalla;
using namespace mmp;


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


const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};


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
                        MapMatcher* matcher,
                        bool verbose)
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
      serialize_geometry_matched_coordinates(results, writer);
    }
  } else {
    serialize_results_as_feature(results, matcher->mapmatching(), writer, route, verbose);
  }

  if (verbose) {
    writer.String("config");
    serialize_config(matcher, writer);
  }

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
        customizable_(ptree_array_to_unordered_set<std::string>(config_.get_child("mm.customizable"))),
        verbose_(config_.get<bool>("mm.verbose")),
        geojson_reader_(config_.get<float>("mm.gps_accuracy"), config_.get<float>("mm.search_radius")) {}

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
        else if (name == "route" || name == "geometry" || name == "verbose") {
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
      std::vector<std::vector<Measurement>> sequences;
      bool is_collection;

      // Parse sequences
      try {
        is_collection = geojson_reader_.Read(request.body, sequences);
      } catch (const SequenceParseError& ex) {
        return jsonify_error(ex.what(), info);
      }

      if (!is_collection) {
        assert(sequences.size() == 1);
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
        // TODO: invalid_argument is ambiguous
      } catch (const std::invalid_argument& ex) {
        return jsonify_error(ex.what(), info);
      }

      std::vector<MatchResult> results;

      // Match
      if (is_collection) {
        // TODO: handle result lists
        std::vector<std::vector<MatchResult>> result_lists;
        for (const auto& sequence: sequences) {
          result_lists.push_back(matcher->OfflineMatch(sequence));
        }
      } else {
        results = matcher->OfflineMatch(sequences.front());
      }

      // Serialize results
      rapidjson::StringBuffer sb;
      bool verbose = preferences.get<bool>("verbose", verbose_);
      serialize_response(sb, results, matcher, verbose);

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
  { matcher_factory_.ClearFullCache(); }

 protected:
  const boost::property_tree::ptree config_;
  MapMatcherFactory matcher_factory_;
  std::unordered_set<std::string> customizable_;
  bool verbose_;
  GeoJSONReader geojson_reader_;
};


}


namespace mmp {

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
