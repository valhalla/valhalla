#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/geojson.h>

#include "thor/service.h"
#include "thor/isochrone.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

  const std::unordered_map<std::string, thor_worker_t::MATRIX_TYPE> MATRIX{
    {"one_to_many", thor_worker_t::ONE_TO_MANY},
    {"many_to_one", thor_worker_t::MANY_TO_ONE},
    {"many_to_many", thor_worker_t::MANY_TO_MANY},
    {"optimized_route", thor_worker_t::OPTIMIZED_ROUTE}
  };

  std::size_t tdindex = 0;
  constexpr double kMilePerMeter = 0.000621371;
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

}

namespace valhalla {
  namespace thor {

    thor_worker_t::thor_worker_t(const boost::property_tree::ptree& config): mode(valhalla::sif::TravelMode::kPedestrian),
      config(config), reader(config.get_child("mjolnir")),
      long_request(config.get<float>("thor.logging.long_request")){
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("transit", sif::CreateTransitCost);
      factory.Register("truck", sif::CreateTruckCost);
    }

    thor_worker_t::~thor_worker_t(){}

    worker_t::result_t thor_worker_t::work(const std::list<zmq::message_t>& job, void* request_info) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Thor Request " + std::to_string(info.id));
      try{
        //get some info about what we need to do
        boost::property_tree::ptree request;
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        try {
          boost::property_tree::read_json(stream, request);
        }
        catch(const std::exception& e) {
          worker_t::result_t result{false};
          http_response_t response(500, "Internal Server Error", "Failed to parse intermediate request format", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          valhalla::midgard::logging::Log("500::" + std::string(e.what()), " [ANALYTICS] ");
          return result;
        }
        catch(...) {
          worker_t::result_t result{false};
          http_response_t response(500, "Internal Server Error", "Failed to parse intermediate request format", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          valhalla::midgard::logging::Log("500::non-std::exception", " [ANALYTICS] ");
          return result;
        }

        // Initialize request - get the PathALgorithm to use
        std::string costing = init_request(request);
        auto date_time_type = request.get_optional<int>("date_time.type");
        auto matrix_type = request.get_optional<std::string>("matrix_type");
        if (matrix_type) {
          auto matrix_iter = MATRIX.find(*matrix_type);
          if (matrix_iter == MATRIX.cend())
            throw std::runtime_error("Incorrect type provided:: " + *matrix_type + "  Accepted types are 'one_to_many', 'many_to_one', 'many_to_many' or 'optimized_route'.");

          switch (matrix_iter->second) {
            case ONE_TO_MANY:
            case MANY_TO_ONE:
            case MANY_TO_MANY:
              valhalla::midgard::logging::Log("matrix_type::" + *matrix_type, " [ANALYTICS] ");
              return matrix(matrix_iter->second, costing, request, info);
            case OPTIMIZED_ROUTE:
              valhalla::midgard::logging::Log("matrix_type::" + *matrix_type, " [ANALYTICS] ");
              return optimized_path(correlated, costing, request_str, info.do_not_track);
          }
        }//TODO: move isochrones logic to separate file
        else if(request.get_optional<bool>("isochrone")) {
          std::vector<float> contours;
          std::vector<std::string> colors;
          for(const auto& contour : request.get_child("contours")) {
            contours.push_back(contour.second.get<float>("time"));
            colors.push_back(contour.second.get<std::string>("color", ""));
          }

          //get the raster
          auto grid = costing == "multimodal" ?
            isochrone.ComputeMultiModal(correlated, contours.back(), reader, mode_costing, mode) :
            isochrone.Compute(correlated, contours.back(), reader, mode_costing, mode);

          //turn it into geojson
          auto isolines = grid->GenerateContours(contours);
          auto geojson = baldr::json::to_geojson<PointLL>(isolines, colors);
          auto id = request.get_optional<std::string>("id");
          if(id)
            geojson->emplace("id", *id);
          std::stringstream stream; stream << *geojson;

          //return the geojson
          worker_t::result_t result{false};
          http_response_t response(200, "OK", stream.str(), headers_t{CORS, JSON_MIME});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }

        //regular route
        return trip_path(costing, request_str, date_time_type, info.do_not_track);
      }
      catch(const std::exception& e) {
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what(), headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return result;
      }
    }

    /**
     * Update the origin edges for a through location.
     */
    void thor_worker_t::update_origin(baldr::PathLocation& origin, bool prior_is_node,
                      const baldr::GraphId& through_edge) {
      if (prior_is_node) {
        // TODO - remove the opposing through edge from list of edges unless
        // all outbound edges are entering noth_thru regions.
        // For now allow all edges
      } else {
        // Check if the edge is entering a not_thru region - if so do not
        // exclude the opposing edge
        const DirectedEdge* de = reader.GetGraphTile(through_edge)->directededge(through_edge);
        if (de->not_thru()) {
          return;
        }

        // Set the origin edge to the through_edge
        for (auto e : origin.edges) {
          if (e.id == through_edge) {
            origin.edges.clear();
            origin.edges.push_back(e);
            break;
          }
        }
      }
    }

    void thor_worker_t::get_path(PathAlgorithm* path_algorithm,
                 baldr::PathLocation& origin, baldr::PathLocation& destination,
                 std::vector<thor::PathInfo>& path_edges) {
      midgard::logging::Log("#_passes::1", " [ANALYTICS] ");
      // Find the path.
      path_edges = path_algorithm->GetBestPath(origin, destination, reader,
                                               mode_costing, mode);
      // If path is not found try again with relaxed limits (if allowed)
      if (path_edges.size() == 0) {
        valhalla::sif::cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
        if (cost->AllowMultiPass()) {
          // 2nd pass. Less aggressive hierarchy transitioning
          path_algorithm->Clear();
          bool using_astar = (path_algorithm == &astar);
          float relax_factor = using_astar ? 16.0f : 8.0f;
          float expansion_within_factor = using_astar ? 4.0f : 2.0f;
          cost->RelaxHierarchyLimits(relax_factor, expansion_within_factor);
          midgard::logging::Log("#_passes::2", " [ANALYTICS] ");
          path_edges = path_algorithm->GetBestPath(origin, destination,
                                    reader, mode_costing, mode);

          // 3rd pass (only for A*)
          if (path_edges.size() == 0 && using_astar) {
            path_algorithm->Clear();
            cost->DisableHighwayTransitions();
            midgard::logging::Log("#_passes::3", " [ANALYTICS] ");
            path_edges = path_algorithm->GetBestPath(origin, destination,
                                     reader, mode_costing, mode);
          }
        }
      }
    }

    // Get the costing options. Get the base options from the config and the
    // options for the specified costing method. Merge in any request costing
    // options.
    valhalla::sif::cost_ptr_t thor_worker_t::get_costing(const boost::property_tree::ptree& request,
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
        boost::property_tree::ptree overridden = *config_costing;
        for (const auto& r : *request_costing) {
          overridden.put_child(r.first, r.second);
        }
        return factory.Create(costing, overridden);
      }
      // No options to override so use the config options verbatim
      return factory.Create(costing, *config_costing);
    }

    std::string thor_worker_t::init_request(const boost::property_tree::ptree& request) {
      auto id = request.get_optional<std::string>("id");
      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      auto units = request.get<std::string>("units", "km");
      if (units == "mi")
        distance_scale = kMilePerMeter;
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
      if(locations.size() < (request.get_optional<bool>("isochrone") ? 1 : 2))
        throw std::runtime_error("Insufficient number of locations provided");

      //type - 0: current, 1: depart, 2: arrive
      auto date_time_type = request.get_optional<int>("date_time.type");
      auto date_time_value = request.get_optional<std::string>("date_time.value");

      if (date_time_type == 0) //current.
        locations.front().date_time_ = "current";
      else if (date_time_type == 1) //depart at
        locations.front().date_time_ = date_time_value;
      else if (date_time_type == 2) //arrive)
        locations.back().date_time_ = date_time_value;

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
      valhalla::midgard::logging::Log("travel_mode::" + std::to_string(static_cast<uint32_t>(mode)), " [ANALYTICS] ");
      return *costing;
    }

    void thor_worker_t::cleanup() {
      astar.Clear();
      bidir_astar.Clear();
      multi_modal_astar.Clear();
      locations.clear();
      correlated.clear();
      isochrone.Clear();
      if(reader.OverCommitted())
        reader.Clear();
    }

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
