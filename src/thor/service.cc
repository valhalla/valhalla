#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

#include "thor/service.h"
#include "thor/trippathbuilder.h"
#include "thor/pathalgorithm.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


namespace {
  //TODO: throw this in the header to make it testable?
  class thor_worker_t {
   public:
    thor_worker_t(const boost::property_tree::ptree& config): config(config), reader(config.get_child("mjolnir.hierarchy")) {
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("transit", sif::CreateTransitCost);
    }
    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Thor Request " + std::to_string(info.id));
      try{
        //get some info about what we need to do
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        boost::property_tree::ptree request;
        boost::property_tree::read_info(stream, request);

        // Initialize request - check if multimodal
        bool multimodal = init_request(request);
        worker_t::result_t result{true};
        // Forward the original request
        result.messages.emplace_back(std::move(request_str));

        // For each pair of origin/destination
        for(auto path_location = ++correlated.cbegin(); path_location != correlated.cend(); ++path_location) {
          auto origin = *std::prev(path_location);
          auto destination = *path_location;
          // Find the path. Multimodal is a separate case.
          std::vector<thor::PathInfo> path_edges;
          if (multimodal) {
            path_edges = path_algorithm.GetBestPathMM(origin, destination, reader, mode_costing);
            if (path_edges.size() == 0) {
              throw std::runtime_error("No path could be found for input");
            }
          } else {
            //find a path
            path_edges = path_algorithm.GetBestPath(origin, destination, reader, cost);
            if (path_edges.size() == 0) {
              if (cost->AllowMultiPass()) {
                LOG_INFO("Try again with relaxed hierarchy limits");
                path_algorithm.Clear();
                cost->RelaxHierarchyLimits(16.0f);
                path_edges = path_algorithm.GetBestPath(origin, destination, reader, cost);
              }
            }
            if (path_edges.size() == 0) {
              path_algorithm.Clear();
              cost->DisableHighwayTransitions();
              path_edges = path_algorithm.GetBestPath(origin, destination, reader, cost);
              if (path_edges.size() == 0) {
                throw std::runtime_error("No path could be found for input");
              }
            }
          }

          // Form output information based on path edges
          auto trip_path = thor::TripPathBuilder::Build(reader, path_edges, origin, destination);
          // The protobuf path
          result.messages.emplace_back(trip_path.SerializeAsString());
        }

        return result;
      }
      catch(const std::exception& e) {
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what());
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
    }

    // Get the costing options. Get the base options from the config and the
    // options for the specified costing method. Merge in any request costing
    // options.
    valhalla::sif::cost_ptr_t get_costing(const boost::property_tree::ptree& request,
                                          const std::string& costing) {
      std::string method_options = "costing_options." + costing;
      boost::property_tree::ptree config_costing = config.get_child(method_options);
      const auto& request_costing = request.get_child_optional(method_options);
      if (request_costing) {
        // If the request has any options for this costing type, merge the 2
        // costing options - override any config options that are in the request.
        // and add any request options not in the config.
        for (const auto& r : *request_costing) {
          config_costing.put_child(r.first, r.second);
        }
      }
      return factory.Create(costing, config_costing);
    }

    bool init_request(const boost::property_tree::ptree& request) {
      //we require locations
      try {
        for(const auto& location : request.get_child("locations"))
          locations.push_back(baldr::Location::FromPtree(location.second));
        if(locations.size() < 2)
          throw;
        //TODO: bail if this is too many
      }
      catch(...) {
        throw std::runtime_error("insufficiently specified required parameter 'locations'");
      }

      //we require correlated locations
      try {
        size_t i = 0;
        do {
          auto path_location = request.get_child_optional("correlated_" + std::to_string(i));
          if(!path_location)
            break;
          correlated.emplace_back(PathLocation::FromPtree(locations, *path_location));
        }while(++i);
      }
      catch(...) {
        throw std::runtime_error("path computation requires graph correlated locations");
      }

      // Parse out the type of route - this provides the costing method to use
      std::string costing;
      try {
        costing = request.get<std::string>("costing");
      }
      catch(...) {
        throw std::runtime_error("No edge/node costing provided");
      }

      // Construct costing. For multi-modal we construct costing for all modes
      if (costing == "multimodal") {
        mode_costing[0] = get_costing(request, "auto");
        mode_costing[1] = get_costing(request, "pedestrian");
        mode_costing[2] = get_costing(request, "bicycle");
        mode_costing[3] = get_costing(request, "transit");
        return true;
      } else {
        cost = get_costing(request, costing);
        return false;
      }
    }
    void cleanup() {
      path_algorithm.Clear();
      locations.clear();
      if(reader.OverCommitted())
        reader.Clear();
    }
   protected:
    boost::property_tree::ptree config;
    std::vector<Location> locations;
    std::vector<PathLocation> correlated;
    sif::CostFactory<sif::DynamicCost> factory;
    valhalla::sif::cost_ptr_t cost;
    valhalla::sif::cost_ptr_t mode_costing[4];    // TODO - max # of modes?
    valhalla::baldr::GraphReader reader;
    valhalla::thor::PathAlgorithm path_algorithm;
  };
}

namespace valhalla {
  namespace thor {
    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from thor proxy
      auto upstream_endpoint = config.get<std::string>("thor.service.proxy") + "_out";
      //sends them on to odin
      auto downstream_endpoint = config.get<std::string>("odin.service.proxy") + "_in";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");

      //listen for requests
      zmq::context_t context;
      thor_worker_t thor_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
        std::bind(&thor_worker_t::work, std::ref(thor_worker), std::placeholders::_1, std::placeholders::_2),
        std::bind(&thor_worker_t::cleanup, std::ref(thor_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
  }
}
