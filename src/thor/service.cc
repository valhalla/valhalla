#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
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
#include "thor/timedistancematrix.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;


namespace {
  enum MATRIX_TYPE {  ONE_TO_MANY, MANY_TO_ONE, MANY_TO_MANY };
  const std::unordered_map<std::string, MATRIX_TYPE> MATRIX{
    {"one_to_many", ONE_TO_MANY},
    {"many_to_one", MANY_TO_ONE},
    {"many_to_many", MANY_TO_MANY}
  };
  std::size_t tdindex = 0;
  constexpr double kKmPerMeter = 0.001;
  constexpr double kMilePerMeter = 0.000621371;
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  json::ArrayPtr locations(const std::vector<PathLocation>& correlated) {
    auto input_locs = json::array({});
    for(size_t i = 0; i < correlated.size(); i++) {
      input_locs->emplace_back(
        json::map({
          {"lat", json::fp_t{correlated[i].latlng_.lat(), 6}},
          {"lon", json::fp_t{correlated[i].latlng_.lng(), 6}}
        })
      );
    }
    return input_locs;
  }

  json::ArrayPtr serialize_row(const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds,
      const size_t origin, const size_t destination, const size_t start, const size_t end, double distance_scale) {
    auto row = json::array({});
    for(size_t i = start; i < end; i++) {
      //check to make sure a route was found; if not, return null for distance & time in matrix result
      if (tds[i].time != kMaxCost) {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(origin)},
          {"to_index", static_cast<uint64_t>(destination + (i - start))},
          {"time", static_cast<uint64_t>(tds[i].time)},
          {"distance", json::fp_t{tds[i].dist * distance_scale, 3}}
        }));
      } else {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(origin)},
          {"to_index", static_cast<uint64_t>(destination + (i - start))},
          {"time", static_cast<nullptr_t>(nullptr)},
          {"distance", static_cast<nullptr_t>(nullptr)}
        }));
      }
    }
    return row;
  }

  //Returns a row vector of computed time and distance from the first (origin) location to each additional location provided.
  // {
  //   input_locations: [{},{},{}],
  //   one_to_many:
  //   [
  //     [{origin0,dest0,0,0},{origin0,dest1,x,x},{origin0,dest2,x,x},{origin0,dest3,x,x}]
  //   ]
  // }
  json::MapPtr serialize_one_to_many(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
     auto json = json::map({
      {"one_to_many", json::array({serialize_row(correlated, tds, 0, 0, 0, tds.size(), distance_scale)})},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

  //Returns a column vector of computed time and distance from each location to the last (destination) location provided.
  // {
  //   input_locations: [{},{},{}],
  //   many_to_one:
  //   [
  //     [{origin0,dest0,x,x}],
  //     [{origin1,dest0,x,x}],
  //     [{origin2,dest0,x,x}],
  //     [{origin3,dest0,0,0}]
  //   ]
  // }
  json::MapPtr serialize_many_to_one(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr column_matrix = json::array({});
    for(size_t i = 0; i < correlated.size(); ++i)
      column_matrix->emplace_back(serialize_row(correlated, tds, i, correlated.size() - 1, i, i + 1, distance_scale));
    auto json = json::map({
      {"many_to_one", column_matrix},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

  //Returns a square matrix of computed time and distance from each location to every other location.
  // {
  //   input_locations: [{},{},{}],
  //   many_to_many:
  //   [
  //     [{origin0,dest0,0,0},{origin0,dest1,x,x},{origin0,dest2,x,x},{origin0,dest3,x,x}],
  //     [{origin1,dest0,x,x},{origin1,dest1,0,0},{origin1,dest2,x,x},{origin1,dest3,x,x}],
  //     [{origin2,dest0,x,x},{origin2,dest1,x,x},{origin2,dest2,0,0},{origin2,dest3,x,x}],
  //     [{origin3,dest0,x,x},{origin3,dest1,x,x},{origin3,dest2,x,x},{origin3,dest3,0,0}]
  //   ]
  // }
  json::MapPtr serialize_many_to_many(const boost::optional<std::string>& id, const std::vector<PathLocation>& correlated, const std::vector<TimeDistance>& tds, std::string& units, double distance_scale) {
    json::ArrayPtr square_matrix = json::array({});
    for(size_t i = 0; i < correlated.size(); ++i)
      square_matrix->emplace_back(serialize_row(correlated, tds, i, 0, correlated.size() * i, correlated.size() * (i + 1), distance_scale));
    auto json = json::map({
      {"many_to_many", square_matrix},
      {"locations", json::array({locations(correlated)})},
      {"units", units},
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }

  //TODO: throw this in the header to make it testable?
  class thor_worker_t {
   public:
    thor_worker_t(const boost::property_tree::ptree& config): mode(valhalla::sif::TravelMode::kPedestrian),
      config(config), reader(config.get_child("mjolnir.hierarchy")),
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

    std::string init_request(boost::property_tree::ptree& request) {
      //get time for start of request
      auto start_time = std::chrono::high_resolution_clock::now();
      auto msecs = std::chrono::duration_cast<std::chrono::milliseconds>(start_time.time_since_epoch()).count();
      request.put("thor_start_time", msecs);

      auto id = request.get_optional<std::string>("id");

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
      return *costing;
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
        auto matrix = request.get_optional<std::string>("matrix_type");
        if (matrix) {
          valhalla::midgard::logging::Log("matrix_type::" + *matrix, " [ANALYTICS] ");
          auto matrix_iter = MATRIX.find(*matrix);
          if (matrix_iter != MATRIX.cend()) {
            return get_matrix(matrix_iter->second, costing, request, info);
          }
          else { //this will never happen since loki formats the request for matrix
            throw std::runtime_error("Incorrect matrix_type provided:: " + *matrix + "  Accepted types are 'one_to_many', 'many_to_one' or 'many_to_many'.");
          }
        }
        return get_trip_path(costing, request_str, date_time_type);
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

    worker_t::result_t get_trip_path(const std::string &costing, const std::string &request_str, boost::optional<int> &date_time_type){
      worker_t::result_t result{true};

      // Forward the original request
      result.messages.emplace_back(std::move(request_str));

      // For each pair of origin/destination
      bool prior_is_node = false;
      baldr::GraphId through_edge;
      std::vector<baldr::PathLocation> through_loc;
      std::vector<thor::PathInfo> path_edges;
      std::string origin_date_time, dest_date_time;

      if (date_time_type && *date_time_type == 2) {

        std::list<std::string> messages;
        baldr::PathLocation& last_break_dest = *correlated.rbegin();

        for(auto path_location = ++correlated.crbegin(); path_location != correlated.crend(); ++path_location) {
          auto origin = *path_location;
          auto destination = *std::prev(path_location);

          // Through edge is valid if last orgin was "through"
          if (through_edge.Is_Valid()) {
            UpdateOrigin(origin, prior_is_node, through_edge);
          } else {
            last_break_dest = destination;
          }

          // Get the algorithm type for this location pair
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

          // Get best path
          if (path_edges.size() == 0) {
            GetPath(path_algorithm, origin, destination, path_edges);
            if (path_edges.size() == 0) {
              throw std::runtime_error("No path could be found for input");
            }
          } else {
            // Get the path in a temporary vector
            std::vector<thor::PathInfo> temp_path;
            GetPath(path_algorithm, origin, destination, temp_path);
            if (temp_path.size() == 0) {
              throw std::runtime_error("No path could be found for input");
            }

            // Append the temp_path edges to path_edges, adding the elapsed
            // time from the end of the current path. If continuing along the
            // same edge, remove the prior so we do not get a duplicate edge.
            uint32_t t = path_edges.back().elapsed_time;
            if (temp_path.front().edgeid == path_edges.back().edgeid) {
              path_edges.pop_back();
            }
            for (auto edge : temp_path) {
              edge.elapsed_time += t;
              path_edges.emplace_back(edge);
            }
          }

          // Build trip path for this leg and add to the result if this
          // location is a BREAK or if this is the last location
          if (origin.stoptype_ == Location::StopType::BREAK ||
              path_location == --correlated.crend()) {

              if (!origin_date_time.empty())
                last_break_dest.date_time_ = origin_date_time;

              // Form output information based on path edges
              auto trip_path = thor::TripPathBuilder::Build(reader, path_edges,
                                  origin, last_break_dest, through_loc);

              if (origin.date_time_)
                origin_date_time = *origin.date_time_;

              // The protobuf path
              messages.emplace_front(trip_path.SerializeAsString());

              // Clear path edges and set through edge to invalid
              path_edges.clear();
              through_edge = baldr::GraphId();
          } else {
              // This is a through location. Save last edge as the through_edge
              prior_is_node = origin.IsNode();
              through_edge = path_edges.back().edgeid;

              // Add to list of through locations for this leg
              through_loc.emplace_back(origin);
          }

          // If we have another one coming we need to clear
          if (--correlated.crend() != path_location)
            path_algorithm->Clear();
        }

        for (const auto msg : messages)
          result.messages.emplace_back(msg);

      } else {
        baldr::PathLocation& last_break_origin = correlated[0];
        for(auto path_location = ++correlated.cbegin(); path_location != correlated.cend(); ++path_location) {
          auto origin = *std::prev(path_location);
          auto destination = *path_location;

          if (date_time_type && (*date_time_type == 0 || *date_time_type == 1) &&
              !dest_date_time.empty() && origin.stoptype_ == Location::StopType::BREAK)
            origin.date_time_ = dest_date_time;

          // Through edge is valid if last destination was "through"
          if (through_edge.Is_Valid()) {
            UpdateOrigin(origin, prior_is_node, through_edge);
          } else {
            last_break_origin = origin;
          }

          // Get the algorithm type for this location pair
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

          // Get best path
          if (path_edges.size() == 0) {
            GetPath(path_algorithm, origin, destination, path_edges);
            if (path_edges.size() == 0) {
              throw std::runtime_error("No path could be found for input");
            }

            if (date_time_type && *date_time_type == 0 && origin_date_time.empty() &&
                origin.stoptype_ == Location::StopType::BREAK)
              last_break_origin.date_time_ = origin.date_time_;

          } else {
            // Get the path in a temporary vector
            std::vector<thor::PathInfo> temp_path;
            GetPath(path_algorithm, origin, destination, temp_path);
            if (temp_path.size() == 0) {
              throw std::runtime_error("No path could be found for input");
            }

            if (date_time_type && *date_time_type == 0 && origin_date_time.empty() &&
                origin.stoptype_ == Location::StopType::BREAK)
              last_break_origin.date_time_ = origin.date_time_;

            // Append the temp_path edges to path_edges, adding the elapsed
            // time from the end of the current path. If continuing along the
            // same edge, remove the prior so we do not get a duplicate edge.
            uint32_t t = path_edges.back().elapsed_time;
            if (temp_path.front().edgeid == path_edges.back().edgeid) {
              path_edges.pop_back();
            }
            for (auto edge : temp_path) {
              edge.elapsed_time += t;
              path_edges.emplace_back(edge);
            }
          }

          // Build trip path for this leg and add to the result if this
          // location is a BREAK or if this is the last location
          if (destination.stoptype_ == Location::StopType::BREAK ||
              path_location == --correlated.cend()) {
              // Form output information based on path edges
              auto trip_path = thor::TripPathBuilder::Build(reader, path_edges,
                                                            last_break_origin, destination, through_loc);

              if (date_time_type) {
                origin_date_time = *last_break_origin.date_time_;
                dest_date_time = *destination.date_time_;
              }

              // The protobuf path
              result.messages.emplace_back(trip_path.SerializeAsString());

              // Clear path edges and set through edge to invalid
              path_edges.clear();
              through_edge = baldr::GraphId();
          } else {
              // This is a through location. Save last edge as the through_edge
              prior_is_node = destination.IsNode();
              through_edge = path_edges.back().edgeid;

              // Add to list of through locations for this leg
              through_loc.emplace_back(destination);
          }

          // If we have another one coming we need to clear
          if (--correlated.cend() != path_location)
            path_algorithm->Clear();
        }
      }
      return result;
    }

    /**
     * Update the origin edges for a through location.
     */
    void UpdateOrigin(baldr::PathLocation& origin, bool prior_is_node,
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
        auto edges = origin.edges();
        for (auto e : edges) {
          if (e.id == through_edge) {
            origin.ClearEdges();
            origin.CorrelateEdge(e);
            break;
          }
        }
      }
    }

    void GetPath(thor::PathAlgorithm* path_algorithm,
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
          // 2nd pass
          path_algorithm->Clear();
          cost->RelaxHierarchyLimits(16.0f);
          midgard::logging::Log("#_passes::2", " [ANALYTICS] ");
          path_edges = path_algorithm->GetBestPath(origin, destination,
                                    reader, mode_costing, mode);

          // 3rd pass
          if (path_edges.size() == 0) {
            path_algorithm->Clear();
            cost->DisableHighwayTransitions();
            midgard::logging::Log("#_passes::3", " [ANALYTICS] ");
            path_edges = path_algorithm->GetBestPath(origin, destination,
                                     reader, mode_costing, mode);
          }
        }
      }
    }

    worker_t::result_t  get_matrix(const MATRIX_TYPE matrix_type, const std::string &costing, const boost::property_tree::ptree &request, http_request_t::info_t& request_info) {
      // Parse out units; if none specified, use kilometers
      double distance_scale = kKmPerMeter;
      auto units = request.get<std::string>("units", "km");
      if (units == "mi")
        distance_scale = kMilePerMeter;
      else {
        units = "km";
        distance_scale = kKmPerMeter;
      }

      //do the real work
      json::MapPtr json;
      thor::TimeDistanceMatrix tdmatrix;
      switch ( matrix_type) {
       case MATRIX_TYPE::ONE_TO_MANY:
         json = serialize_one_to_many(request.get_optional<std::string>("id"), correlated, tdmatrix.OneToMany(0, correlated, reader, mode_costing, mode), units, distance_scale);
         break;
       case MATRIX_TYPE::MANY_TO_ONE:
         json = serialize_many_to_one(request.get_optional<std::string>("id"), correlated, tdmatrix.ManyToOne(correlated.size() - 1, correlated, reader, mode_costing, mode), units, distance_scale);
         break;
       case MATRIX_TYPE::MANY_TO_MANY:
         json = serialize_many_to_many(request.get_optional<std::string>("id"), correlated, tdmatrix.ManyToMany(correlated, reader, mode_costing, mode), units, distance_scale);
         break;
      }

      //get processing time for matrix
      auto end_time = std::chrono::high_resolution_clock::now();
      auto msecs = std::chrono::duration_cast<std::chrono::milliseconds>(end_time.time_since_epoch()).count();
      auto elapsed_time = static_cast<float>(msecs - request.get<size_t>("thor_start_time"));

      //log request if greater than X (ms)
      if ((elapsed_time / correlated.size()) > long_request) {
        std::stringstream ss;
        boost::property_tree::json_parser::write_json(ss, request, false);
        LOG_WARN("matrix request elapsed time (ms)::"+ std::to_string(elapsed_time));
        LOG_WARN("matrix request exceeded threshold::"+ ss.str());
        midgard::logging::Log("thor_long_request", " [ANALYTICS] ");
      }

      //jsonp callback if need be
      std::ostringstream stream;
      auto jsonp = request.get_optional<std::string>("jsonp");
      if(jsonp)
        stream << *jsonp << '(';
      stream << *json;
      if(jsonp)
        stream << ')';

      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      worker_t::result_t result{false};
      result.messages.emplace_back(response.to_string());
      return result;
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
        boost::property_tree::ptree overridden = *config_costing;
        for (const auto& r : *request_costing) {
          overridden.put_child(r.first, r.second);
        }
        return factory.Create(costing, overridden);
      }
      // No options to override so use the config options verbatim
      return factory.Create(costing, *config_costing);
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
    float long_request;
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
