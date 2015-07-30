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
#include "thor/bidirectional_astar.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};

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
        boost::property_tree::ptree request;
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        try {
          boost::property_tree::read_info(stream, request);
        }
        catch(...) {
          worker_t::result_t result{false};
          http_response_t response(500, "Internal Server Error", "Failed to parse intermediate request format", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }

        // Initialize request - get the PathALgorithm to use
        std::string costing = init_request(request);
        worker_t::result_t result{true};

        // Forward the original request
        result.messages.emplace_back(std::move(request_str));

        // For each pair of origin/destination
        for(auto path_location = ++correlated.cbegin(); path_location != correlated.cend(); ++path_location) {
          auto origin = *std::prev(path_location);
          auto destination = *path_location;

          // Get the algorithm type
          thor::PathAlgorithm* path_algorithm;
          if (costing == "multimodal") {
            path_algorithm = &multi_modal_astar;
          } else if (costing == "pedestrian" || costing == "bicycle") {
            // Use bidirectional A* for pedestrian and bicycle if over 10km
            float dist = origin.latlng_.Distance(destination.latlng_);
            path_algorithm = (dist > 10000.0f) ? &bidir_astar : &astar;
          } else {
            path_algorithm = &astar;
          }

          // Find the path.
          std::vector<thor::PathInfo> path_edges = path_algorithm->GetBestPath(
                  origin, destination, reader, mode_costing, mode);

          // If path is not found try again with relaxed limits (if allowed)
          if (path_edges.size() == 0) {
            valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
            if (cost->AllowMultiPass()) {
              // 2nd pass
              LOG_INFO("Try again with relaxed hierarchy limits");
              path_algorithm->Clear();
              cost->RelaxHierarchyLimits(16.0f);
              path_edges = path_algorithm->GetBestPath(origin, destination, reader, mode_costing, mode);

              // 3rd pass
              if (path_edges.size() == 0) {
                path_algorithm->Clear();
                cost->DisableHighwayTransitions();
                path_edges = path_algorithm->GetBestPath(origin, destination, reader, mode_costing, mode);
              }
            }
          }

          if (path_edges.size() == 0) {
            throw std::runtime_error("No path could be found for input");
          }

          // Form output information based on path edges
          auto trip_path = thor::TripPathBuilder::Build(reader, path_edges, origin, destination);
          // The protobuf path
          result.messages.emplace_back(trip_path.SerializeAsString());

          //if we have another one coming we need to clear
          if(--correlated.cend() != path_location)
            path_algorithm->Clear();
        }

        return result;
      }
      catch(const std::exception& e) {
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what(), headers_t{CORS});
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
      auto config_costing = config.get_child_optional(method_options);
      if(!config_costing)
        throw std::runtime_error("No costing method found for '" + costing + "'");
      auto request_costing = request.get_child_optional(method_options);
      if(request_costing) {
        // If the request has any options for this costing type, merge the 2
        // costing options - override any config options that are in the request.
        // and add any request options not in the config.
        for (const auto& r : *request_costing) {
          config_costing->put_child(r.first, r.second);
        }
      }
      return factory.Create(costing, *config_costing);
    }

    std::string init_request(const boost::property_tree::ptree& request) {
      //we require locations
        auto request_locations = request.get_child_optional("locations");
        if(!request_locations)
          throw std::runtime_error("Insufficiently specified required parameter 'locations'");
        for(const auto& location : *request_locations) {
          try{
            locations.push_back(baldr::Location::FromPtree(location.second));
          }
          catch (...) {
            throw std::runtime_error("Failed to parse location");
          }
        }
        if(locations.size() < 2)
          throw std::runtime_error("Insufficient number of locations provided");

      //we require correlated locations
      size_t i = 0;
      do {
        auto path_location = request.get_child_optional("correlated_" + std::to_string(i));
        if(!path_location)
          break;
        try {
          correlated.emplace_back(PathLocation::FromPtree(locations, *path_location));
        }
        catch (...) {
          throw std::runtime_error("Failed to parse correlated location");
        }
      }while(++i);

      // Parse out the type of route - this provides the costing method to use
      auto costing = request.get_optional<std::string>("costing");
      if(!costing)
        throw std::runtime_error("No edge/node costing provided");

      // Set travel mode and construct costing
      if (*costing == "multimodal") {
        // For multi-modal we construct costing for all modes and set the
        // initial mode to pedestrian. (TODO - allow other initial modes)
        mode_costing[0] = get_costing(request, "auto");
        mode_costing[1] = get_costing(request, "pedestrian");
        mode_costing[2] = get_costing(request, "bicycle");
        mode_costing[3] = get_costing(request, "transit");
        mode = valhalla::sif::TravelMode::kPedestrian;
      } else {
        valhalla::sif::cost_ptr_t cost = get_costing(request, *costing);
        mode = cost->travelmode();
        mode_costing[static_cast<uint32_t>(mode)] = cost;
      }
      return *costing;
    }

    void cleanup() {
      astar.Clear();
      bidir_astar.Clear();
      multi_modal_astar.Clear();
      locations.clear();
      correlated.clear();
      if(reader.OverCommitted())
        reader.Clear();
    }
   protected:
    valhalla::sif::TravelMode mode;
    boost::property_tree::ptree config;
    std::vector<Location> locations;
    std::vector<PathLocation> correlated;
    sif::CostFactory<sif::DynamicCost> factory;
    valhalla::sif::cost_ptr_t mode_costing[4];    // TODO - max # of modes?
    valhalla::baldr::GraphReader reader;

    // Path algorithms (TODO - perhaps use a map?))
    thor::PathAlgorithm astar;
    thor::BidirectionalAStar bidir_astar;
    thor::MultiModalPathAlgorithm multi_modal_astar;
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
