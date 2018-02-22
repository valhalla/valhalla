#include "tyr/actor.h"
#include "exception.h"
#include "loki/worker.h"
#include "thor/worker.h"
#include "odin/worker.h"
#include "tyr/serializers.h"
#include "baldr/rapidjson_utils.h"

#include <boost/property_tree/json_parser.hpp>

using namespace valhalla;
using namespace valhalla::loki;
using namespace valhalla::thor;
using namespace valhalla::odin;

namespace {
  rapidjson::Document to_document(const std::string& request) {
    rapidjson::Document d;
    auto& allocator = d.GetAllocator();
    d.Parse(request.c_str());
    if (d.HasParseError())
      throw valhalla_exception_t{100};
    return d;
  }
}

namespace valhalla {
  namespace tyr {

    struct actor_t::pimpl_t {
      pimpl_t(const boost::property_tree::ptree& config):
        loki_worker(config), thor_worker(config), odin_worker(config) {
      }
      void set_interrupts(const std::function<void ()>& interrupt_function) {
        loki_worker.set_interrupt(interrupt_function);
        thor_worker.set_interrupt(interrupt_function);
        odin_worker.set_interrupt(interrupt_function);
      }
      void cleanup() {
        loki_worker.cleanup();
        thor_worker.cleanup();
        odin_worker.cleanup();
      }
      loki::loki_worker_t loki_worker;
      thor::thor_worker_t thor_worker;
      odin::odin_worker_t odin_worker;
    };

    actor_t::actor_t(const boost::property_tree::ptree& config, bool auto_cleanup): pimpl(new pimpl_t(config)), auto_cleanup(auto_cleanup) {
    }

    void actor_t::cleanup() {
      pimpl->cleanup();
    }

    std::string actor_t::route(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.route(request);
      //route between the locations in the graph to find the best path
      auto date_time_type = rapidjson::get_optional<int>(request, "/date_time.type");
      auto legs = pimpl->thor_worker.route(request, date_time_type);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(legs);
      //serialize them out to json string
      auto bytes = tyr::serializeDirections(options, legs, directions);
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return bytes;
    }

    std::string actor_t::locate(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      auto json = pimpl->loki_worker.locate(request);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::matrix(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.matrix(SOURCES_TO_TARGETS, request);
      //compute the matrix
      auto json = pimpl->thor_worker.matrix(SOURCES_TO_TARGETS, request);
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return json;
    }

    std::string actor_t::optimized_route(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.matrix(OPTIMIZED_ROUTE, request);
      //compute compute all pairs and then the shortest path through them all
      auto legs = pimpl->thor_worker.optimized_route(request);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(legs);
      //serialize them out to json string
      auto bytes = tyr::serializeDirections(options, legs, directions);
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return bytes;
    }

    std::string actor_t::isochrone(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.isochrones(request);
      //compute the isochrones
      auto json = pimpl->thor_worker.isochrones(request);
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return json;
    }

    std::string actor_t::trace_route(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.trace(TRACE_ROUTE, request);
      //route between the locations in the graph to find the best path
      std::list<TripPath> legs{pimpl->thor_worker.trace_route(request)};
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(legs);
      //serialize them out to json string
      auto bytes = tyr::serializeDirections(options, legs, directions);
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return bytes;
    }

    std::string actor_t::trace_attributes(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.trace(TRACE_ATTRIBUTES, request);
      //get the path and turn it into attribution along it
      auto json = pimpl->thor_worker.trace_attributes(request);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::height(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      auto options = from_json(request);
      //get the height at each point
      auto json = pimpl->loki_worker.height(request);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::transit_available(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      //check the request and locate the locations in the graph
      auto json = pimpl->loki_worker.transit_available(request);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
   }

  }
}
