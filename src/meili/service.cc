#include <string>
#include <vector>
#include <thread>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include "midgard/logging.h"
#include "baldr/graphreader.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

#include "meili/universal_cost.h"
#include "meili/geojson_reader.h"
#include "meili/geojson_writer.h"
#include "meili/map_matcher_factory.h"
#include "meili/map_matcher.h"
#include "meili/match_result.h"


using namespace prime_server;
using namespace valhalla;
using namespace meili;


namespace {

// Credits: http://werkzeug.pocoo.org/
const std::unordered_map<size_t, std::string> kHttpStatusCodes {
  // 1xx
  {100,"Continue"},
  {101,"Switching Protocols"},
  {102,"Processing"},

  // 2xx
  {200,"OK"},
  {201,"Created"},
  {202,"Accepted"},
  {203,"Non Authoritative Information"},
  {204,"No Content"},
  {205,"Reset Content"},
  {206,"Partial Content"},
  {207,"Multi Status"},
  {226,"IM Used"},  // see RFC 322

  // 3xx
  {300,"Multiple Choices"},
  {301,"Moved Permanently"},
  {302,"Found"},
  {303,"See Other"},
  {304,"Not Modified"},
  {305,"Use Proxy"},
  {307,"Temporary Redirect"},

  // 4xx
  {400,"Bad Request"},
  {401,"Unauthorized"},
  {402,"Payment Required"},  // unuse
  {403,"Forbidden"},
  {404,"Not Found"},
  {405,"Method Not Allowed"},
  {406,"Not Acceptable"},
  {407,"Proxy Authentication Required"},
  {408,"Request Timeout"},
  {409,"Conflict"},
  {410,"Gone"},
  {411,"Length Required"},
  {412,"Precondition Failed"},
  {413,"Request Entity Too Large"},
  {414,"Request URI Too Long"},
  {415,"Unsupported Media Type"},
  {416,"Requested Range Not Satisfiable"},
  {417,"Expectation Failed"},
  {418,"I\'m a teapot"},  // see RFC 232
  {422,"Unprocessable Entity"},
  {423,"Locked"},
  {424,"Failed Dependency"},
  {426,"Upgrade Required"},
  {428,"Precondition Required"},  // see RFC 658
  {429,"Too Many Requests"},
  {431,"Request Header Fields Too Large"},
  {449,"Retry With"},  // proprietary MS extensio

  // 5xx
  {500,"Internal Server Error"},
  {501,"Not Implemented"},
  {502,"Bad Gateway"},
  {503,"Service Unavailable"},
  {504,"Gateway Timeout"},
  {505,"HTTP Version Not Supported"},
  {507,"Insufficient Storage"},
  {510,"Not Extended"},
};

inline std::string http_status_code(unsigned code) {
  auto code_itr = kHttpStatusCodes.find(code);
  return code_itr == kHttpStatusCodes.cend() ? "" : code_itr->second;
}


const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};


template <typename buffer_t>
void serialize_config(MapMatcher& matcher,
                      rapidjson::Writer<buffer_t>& writer)
{
  // Property tree -> string
  std::stringstream ss;
  boost::property_tree::json_parser::write_json(ss, matcher.config());
  const auto& str = ss.str();

  // String -> JSON document
  rapidjson::Document document;
  document.Parse(str.c_str());

  // JSON document -> writer stream
  document.Accept(writer);
}


std::string serialize_response(MapMatcher& matcher,
                               const std::vector<std::vector<MatchResult>>& result_lists,
                               bool is_collection,
                               bool verbose)
{
  const auto route = matcher.config().get<bool>("route"),
          geometry = matcher.config().get<bool>("geometry");

  using buffer_t = rapidjson::StringBuffer;
  buffer_t sb;
  rapidjson::Writer<buffer_t> writer(sb);

  writer.StartObject();

  writer.String("status");
  writer.Uint(200);

  writer.String("message");
  // Need the reference otherwise the c string will be invalidated?
  // TODO possible to make a c string version of http_status_code?
  const auto& status = http_status_code(200);
  writer.String(status.c_str());

  writer.String("data");
  GeoJSONWriter<buffer_t>* geojson_writer;
  if (route) {
    geojson_writer = new GeoJSONRouteWriter<buffer_t>(verbose);
  } else {
    geojson_writer = new GeoJSONMatchedPointsWriter<buffer_t>(verbose);
  }
  if (geometry) {
    if (is_collection) {
      geojson_writer->WriteGeometryCollection(writer, matcher, result_lists);
    } else {
      geojson_writer->WriteGeometry(writer, matcher, result_lists.front());
    }
  } else {
    if (is_collection) {
      geojson_writer->WriteFeatureCollection(writer, matcher, result_lists);
    } else {
      geojson_writer->WriteFeature(writer, matcher, result_lists.front());
    }
  }
  delete geojson_writer;

  if (verbose) {
    writer.String("config");
    serialize_config(matcher, writer);
  }

  writer.EndObject();

  return sb.GetString();
}


worker_t::result_t jsonify_error(const std::string& message,
                                 http_request_info_t& info,
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


worker_t::result_t jsonify_success(const std::string& content,
                                   http_request_info_t& info)
{
  worker_t::result_t result{false};
  http_response_t response(200, http_status_code(200), content, headers_t{CORS, JS_MIME});
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
        customizable_(ptree_array_to_unordered_set<std::string>(config_.get_child("meili.customizable"))),
        verbose_(config_.get<bool>("meili.verbose")) {}

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

  worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const worker_t::interrupt_function_t&) {
    auto& info = *static_cast<http_request_info_t*>(request_info);
    LOG_INFO("Got Map Matching Request " + std::to_string(info.id));

    http_request_t request;

    // Parse HTTP
    try {
      request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
    } catch (const std::runtime_error& ex) {
      return jsonify_error(ex.what(), info);
    }

    if (request.method == method_t::POST) {
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

      // Read sequences
      bool is_collection;
      std::vector<std::vector<Measurement>> sequences;
      try {
        GeoJSONReader geojson_reader(matcher->config().get<float>("gps_accuracy"),
                                     matcher->config().get<float>("search_radius"));
        is_collection = geojson_reader.Read(request.body, sequences);
      } catch (const std::invalid_argument& ex) {
        return jsonify_error(ex.what(), info);
      } catch (const SequenceParseError& ex) {
        return jsonify_error(ex.what(), info);
      }

      // Match
      std::vector<std::vector<MatchResult>> result_lists;
      if (is_collection) {
        for (const auto& sequence: sequences) {
          result_lists.push_back(matcher->OfflineMatch(sequence));
        }
      } else {
        result_lists.push_back(matcher->OfflineMatch(sequences.front()));
      }

      // Serialize results
      bool verbose = preferences.get<bool>("verbose", verbose_);
      const auto& content = serialize_response(*matcher, result_lists, is_collection, verbose);

      delete matcher;

      return jsonify_success(content, info);
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
};

}


namespace valhalla {

namespace meili {

void run_service(const boost::property_tree::ptree& config) {
  //gets requests from the http server
  auto upstream_endpoint = config.get<std::string>("meili.service.proxy") + "_out";
  //or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  //listen for requests
  zmq::context_t context;
  mm_worker_t meili_worker(config);
  auto work = std::bind(&mm_worker_t::work, std::ref(meili_worker), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  auto cleanup = std::bind(&mm_worker_t::cleanup, std::ref(meili_worker));
  prime_server::worker_t worker(context, upstream_endpoint, "ipc:///dev/null", loopback_endpoint, interrupt_endpoint, work, cleanup);
  worker.work();

  //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}

}

}
