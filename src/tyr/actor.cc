#include "tyr/actor.h"
#include "exception.h"
#include "loki/worker.h"
#include "thor/worker.h"
#include "odin/worker.h"
#include "skadi/worker.h"
#include "tyr/serializers.h"
#include "baldr/rapidjson_utils.h"

#include <boost/property_tree/json_parser.hpp>

using namespace valhalla;
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

  //TODO: delete this and move everything to rapidjson
  boost::property_tree::ptree to_ptree(const std::string str) {
    std::stringstream ss;
    ss << str;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

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
        loki_worker(config), thor_worker(config), odin_worker(config), skadi_worker(config) {
      }
      void set_interrupts(const std::function<void ()>& interrupt_function) {
        loki_worker.set_interrupt(interrupt_function);
        thor_worker.set_interrupt(interrupt_function);
        odin_worker.set_interrupt(interrupt_function);
        skadi_worker.set_interrupt(interrupt_function);
      }
      void cleanup() {
        loki_worker.cleanup();
        thor_worker.cleanup();
        odin_worker.cleanup();
        skadi_worker.cleanup();
      }
      loki::loki_worker_t loki_worker;
      thor::thor_worker_t thor_worker;
      odin::odin_worker_t odin_worker;
      skadi::skadi_worker_t skadi_worker;
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
      //check the request and locate the locations in the graph
      pimpl->loki_worker.route(request);
      //route between the locations in the graph to find the best path
      auto date_time_type = GetOptionalFromRapidJson<int>(request, "/date_time.type");
      auto request_pt = to_ptree(request);
      auto legs = pimpl->thor_worker.route(request_pt, date_time_type);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs);
      //serialize them out to json string
      auto json = tyr::serializeDirections(ROUTE, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::locate(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
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
      //check the request and locate the locations in the graph
      pimpl->loki_worker.matrix(SOURCES_TO_TARGETS, request);
      auto request_pt = to_ptree(request);
      //compute the matrix
      auto json = pimpl->thor_worker.matrix(SOURCES_TO_TARGETS, request_pt);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::optimized_route(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.matrix(OPTIMIZED_ROUTE, request);
      auto request_pt = to_ptree(request);
      //compute compute all pairs and then the shortest path through them all
      auto legs = pimpl->thor_worker.optimized_route(request_pt);
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs);
      //serialize them out to json string
      auto json = tyr::serializeDirections(ROUTE, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::isochrone(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.isochrones(request);
      auto request_pt = to_ptree(request);
      //compute the isochrones
      auto json = pimpl->thor_worker.isochrones(request_pt);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::trace_route(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.trace(TRACE_ROUTE, request);
      //route between the locations in the graph to find the best path
      auto request_pt = to_ptree(request);
      std::list<TripPath> legs{pimpl->thor_worker.trace_route(request_pt)};
      //get some directions back from them
      auto directions = pimpl->odin_worker.narrate(request_pt, legs);
      //serialize them out to json string
      auto json = tyr::serializeDirections(ROUTE, request_pt, directions);
      std::stringstream ss;
      ss << *json;
      //if they want you do to do the cleanup automatically
      if(auto_cleanup)
        cleanup();
      return ss.str();
    }

    std::string actor_t::trace_attributes(const std::string& request_str, const std::function<void ()>& interrupt) {
      //set the interrupts
      pimpl->set_interrupts(interrupt);
      //parse the request
      auto request = to_document(request_str);
      //check the request and locate the locations in the graph
      pimpl->loki_worker.trace(TRACE_ATTRIBUTES, request);
      //get the path and turn it into attribution along it
      auto request_pt = to_ptree(request);
      auto json = pimpl->thor_worker.trace_attributes(request_pt);
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
      auto request_rj = to_document(request_str);
      //get the height at each point
      auto json = pimpl->skadi_worker.height(request_rj);
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
