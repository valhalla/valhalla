#include "tyr/actor.h"
#include "exception.h"
#include "loki/worker.h"
#include "thor/worker.h"
#include "odin/worker.h"
#include "tyr/serializers.h"
#include "baldr/rapidjson_utils.h"

#include <boost/property_tree/json_parser.hpp>

using namespace valhalla::loki;
using namespace valhalla::thor;
using namespace valhalla::odin;

namespace {
  //TODO: delete this and move everything to rapidjson
  boost::property_tree::ptree to_ptree(const rapidjson::Document& rj) {
    std::stringstream ss;
    ss << rapidjson::to_string(rj);
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

}

namespace valhalla {
  namespace tyr {

    struct actor_t::pimpl_t {
      pimpl_t(const boost::property_tree::ptree& config):
        loki_worker(config), thor_worker(config), odin_worker(config) {
      }
      loki::loki_worker_t loki_worker;
      thor::thor_worker_t thor_worker;
      odin::odin_worker_t odin_worker;
    };

    actor_t::actor_t(const boost::property_tree::ptree& config): pimpl(new pimpl_t(config)) {
    }

    std::string actor_t::route(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      pimpl->loki_worker.route(request);
      //route between the locations in the graph to find the best path
      auto date_time_type = GetOptionalFromRapidJson<int>(request, "date_time.type");
      auto request_pt = to_ptree(request);
      auto legs = pimpl->thor_worker.route(request_pt, date_time_type);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs);
      //serialize them out to json string
      auto action = static_cast<ACTION_TYPE>(GetFromRapidJson<int>(request, "action", 0));
      auto json = tyr::serializeDirections(action, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      return ss.str();
    }

    baldr::json::ArrayPtr actor_t::locate(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      pimpl->loki_worker.locate(request);
      auto request_pt = to_ptree(request);

      return result;
    }

    thor::thor_worker_t actor_t::matrix(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      auto action = static_cast<ACTION_TYPE>(GetFromRapidJson<int>(request, "action", 0));
      pimpl->loki_worker.matrix(action, request);
      auto request_pt = to_ptree(request);

      auto matrix = pimpl->thor_worker.matrix(matrix_type, request_pt);

      return result;
    }

    std::string actor_t::optimized_route(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      auto action = static_cast<ACTION_TYPE>(GetFromRapidJson<int>(request, "action", 0));
      pimpl->loki_worker.matrix(action, request);
      auto request_pt = to_ptree(request);
      auto legs = pimpl->thor_worker.optimized_route(request_pt);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs);
      //serialize them out to json string
      auto json = tyr::serializeDirections(action, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      return ss.str();
    }

    void actor_t::isochrone() { throw valhalla_exception_t{107}; }

    std::string actor_t::trace_route(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      auto action = static_cast<ACTION_TYPE>(GetFromRapidJson<int>(request, "action", 0));
      pimpl->loki_worker.trace_route(action, request);
      auto request_pt = to_ptree(request);
      auto legs = pimpl->thor_worker.trace_route(request_pt);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs)
      //serialize them out to json string
      auto json = tyr::serializeDirections(action, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      return ss.str();
    }

    thor::thor_worker_t actor_t::trace_attributes(const std::string& request_str) {
      //parse the request
      rapidjson::Document request;
      //check the request and locate the locations in the graph
      auto action = static_cast<ACTION_TYPE>(GetFromRapidJson<int>(request, "action", 0));
      pimpl->loki_worker.trace_route(action, request);
      //route between the locations in the graph to find the best path
      auto request_pt = to_ptree(request);
      auto attributes = pimpl->thor_worker.trace_attributes(request_pt);

      return result;
    }

    void actor_t::height() { throw valhalla_exception_t{107}; }

  }
}
