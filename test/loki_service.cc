#include "test.h"
#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "proto/api.pb.h"
#include "proto_conversions.h"

#include <boost/property_tree/ptree.hpp>
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
#include <thread>
#include <unistd.h>

#include "filesystem.h"
#include "loki/worker.h"

using namespace valhalla;
using namespace prime_server;

namespace {

const std::vector<http_request_t> valhalla_requests{
    http_request_t(GET, "/status"),
    http_request_t(OPTIONS, "/route"),
    http_request_t(HEAD, "/route"),
    http_request_t(PUT, "/route"),
    http_request_t(DELETE, "/route"),
    http_request_t(TRACE, "/route"),
    http_request_t(CONNECT, "/route"),
    http_request_t(GET, ""),
    http_request_t(POST, ""),
    http_request_t(GET, "/route?json={"),
    http_request_t(POST, "/route", "{"),
    http_request_t(GET, "/route"),
    http_request_t(POST, "/route"),
    http_request_t(GET, "/optimized_route"),
    http_request_t(POST, "/optimized_route"),
    http_request_t(GET, R"(/locate?json={"locations":[{"lon":0}]})"),
    http_request_t(POST, "/locate", R"({"locations":[{"lon":0}]})"),
    http_request_t(GET, R"(/route?json={"locations":[{"lon":0,"lat":90}]})"),
    http_request_t(POST, "/route", R"({"locations":[{"lon":0,"lat":90}]})"),
    http_request_t(GET, R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90}]})"),
    http_request_t(POST, "/route", R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90}]})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90}]})"),
    http_request_t(POST,
                   "/route",
                   R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90}]})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":-90}], "costing": "pedestrian"})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":-90}], "costing": "pedestrian"})"),
    http_request_t(GET, R"(/locate?json={"locations":[{"lon":0,"lat":90}], "costing": "yak"})"),
    http_request_t(POST, "/locate", R"({"locations":[{"lon":0,"lat":90}], "costing": "yak"})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90}], "costing": "auto"})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90}], "costing": "auto"})"),
    http_request_t(GET, R"(/optimized_route?json={"locations":[{"lon":0,"lat":90}]})"),
    http_request_t(GET, R"(/sources_to_targets?json={"targets":[{"lon":0,"lat":90}]})"),
    http_request_t(GET, R"(/optimized_route?json={"locations":[{"lon":0,"lat":90}]})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":"NONE","lat":90}, {"lon":"NONE","lat":90}]})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":0,"lat":-270}, {"lon":0,"lat":90}]})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":0,"lat":90}, {"lon":0,"lat":90}], "coosting": "NONE"})"),
    http_request_t(GET, R"(/sources_to_targets?json={"sources":[{"lon":0}]})"),
    http_request_t(
        GET,
        R"(/sources_to_targets?json={"sources":[{"lon":0,"lat":90}],"targets":[{"lon":0}]})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":0},{"lon":0,"lat":0}],"costing":"pedestrian","avoid_locations":[{"lon":0,"lat":0}]})"),
    http_request_t(
        POST,
        "/trace_attributes",
        R"({"shape":[{"lat":37.8077440,"lon":-122.4197010},{"lat":37.8077440,"lon":-122.4197560},{"lat":37.8077450,"lon":-122.4198180}],"shape_match":"map_snap","best_paths":0,"costing":"pedestrian","directions_options":{"units":"miles"}})"),
    http_request_t(
        POST,
        "/trace_attributes",
        R"({"shape":[{"lat":37.8077440,"lon":-122.4197010},{"lat":37.8077440,"lon":-122.4197560},{"lat":37.8077450,"lon":-122.4198180}],"shape_match":"map_snap","best_paths":5,"costing":"pedestrian","directions_options":{"units":"miles"}})"),
    http_request_t(POST, "/trace_attributes", R"({"encoded_polyline":
        "mx{ilAdxcupCdJm@v|@rG|n@dEz_AlUng@fMnDlAt}@zTdmAtZvx@`Rr_@~IlUnI`HtDjVnSdOhW|On^|JvXl^dmApGzUjGfYzAtOT~SUdYsFtmAmK~zBkAh`ArAdd@vDng@dEb\\nHvb@bQpp@~IjVbj@ngAjV`q@bL~g@nDjVpVbnBdAfCpeA`yL~CpRnCn]`C~g@l@zUGfx@m@x_AgCxiBe@xl@e@re@yBviCeAvkAe@vaBzArd@jFhb@|ZzgBjEjVzFtZxC`RlEdYz@~I~DxWtTxtA`Gn]fEjV~BzV^dDpBfY\\dZ?fNgDx~BrA~q@xB|^fIp{@lK~|@|T`oBbF|h@re@d_E|EtYvMrdAvCzUxMhaAnStwAnNls@xLjj@tlBr{HxQlt@lEr[jB`\\Gvl@oNjrCaCvm@|@vb@rAl_@~B|]pHvx@j`@lzC|Ez_@~Htn@|DrFzPlhAzFn^zApp@xGziA","shape_match":"map_snap","best_paths":3,"costing":"auto","directions_options":{"units":"miles"}})"),
};

const std::vector<std::pair<uint16_t, std::string>> valhalla_responses{
    {200, "{}"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {405,
     R"({"error_code":101,"error":"Try a POST or GET request instead","status_code":405,"status":"Method Not Allowed"})"},
    {404,
     R"({"error_code":106,"error":"Try any of:'\/locate' '\/route' '\/height' '\/sources_to_targets' '\/optimized_route' '\/isochrone' '\/trace_route' '\/trace_attributes' '\/transit_available' '\/expansion' '\/centroid' '\/status' ","status_code":404,"status":"Not Found"})"},
    {404,
     R"({"error_code":106,"error":"Try any of:'\/locate' '\/route' '\/height' '\/sources_to_targets' '\/optimized_route' '\/isochrone' '\/trace_route' '\/trace_attributes' '\/transit_available' '\/expansion' '\/centroid' '\/status' ","status_code":404,"status":"Not Found"})"},
    {400,
     R"({"error_code":100,"error":"Failed to parse json request","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":100,"error":"Failed to parse json request","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":110,"error":"Insufficiently specified required parameter 'locations'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":110,"error":"Insufficiently specified required parameter 'locations'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":112,"error":"Insufficiently specified required parameter 'locations' or 'sources & targets'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":112,"error":"Insufficiently specified required parameter 'locations' or 'sources & targets'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":130,"error":"Failed to parse location","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":130,"error":"Failed to parse location","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":120,"error":"Insufficient number of locations provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":120,"error":"Insufficient number of locations provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":124,"error":"No edge\/node costing provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":124,"error":"No edge\/node costing provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":124,"error":"No edge\/node costing provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":124,"error":"No edge\/node costing provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":154,"error":"Path distance exceeds the max distance limit","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":154,"error":"Path distance exceeds the max distance limit","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":125,"error":"No costing method found:'yak'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":125,"error":"No costing method found:'yak'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":150,"error":"Exceeded max locations:20","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":150,"error":"Exceeded max locations:20","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":120,"error":"Insufficient number of locations provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":112,"error":"Insufficiently specified required parameter 'locations' or 'sources & targets'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":120,"error":"Insufficient number of locations provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":130,"error":"Failed to parse location","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":130,"error":"Failed to parse location","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":124,"error":"No edge\/node costing provided","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":131,"error":"Failed to parse source","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":132,"error":"Failed to parse target","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":157,"error":"Exceeded max avoid locations:0","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":158,"error":"Input trace option is out of bounds:(0). The best_paths lower limit is 1","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":158,"error":"Input trace option is out of bounds:(5). The best_paths upper limit is 4","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":153,"error":"Too many shape points:(102). The best paths shape limit is 100","status_code":400,"status":"Bad Request"})"}};

const std::vector<http_request_t> osrm_requests{
    http_request_t(GET, R"(/status?json={"format":"osrm"})"),
    http_request_t(GET, R"(/route?json={"directions_options":{"format":"osrm"}})"),
    http_request_t(POST, "/route", R"({"directions_options":{"format":"osrm"}})"),
    http_request_t(GET, R"(/optimized_route?json={"directions_options":{"format":"osrm"}})"),
    http_request_t(POST, "/optimized_route", R"({"directions_options":{"format":"osrm"}})"),
    http_request_t(GET,
                   R"(/locate?json={"locations":[{"lon":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(POST,
                   "/locate",
                   R"({"locations":[{"lon":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(POST,
                   "/route",
                   R"({"locations":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":-90}], "costing": "pedestrian","directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":-90}], "costing": "pedestrian","directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/locate?json={"locations":[{"lon":0,"lat":90}], "costing": "yak","directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/locate",
        R"({"locations":[{"lon":0,"lat":90}], "costing": "yak","directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90}], "costing": "auto","directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/route",
        R"({"locations":[{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},{"lon":0,"lat":90},
        {"lon":0,"lat":90},{"lon":0,"lat":90}], "costing": "auto","directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"sources":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"targets":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/sources_to_targets?json={"targets":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/sources_to_targets?json={"locations":[{"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":"NONE","lat":90}, {"lon":"NONE","lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":0,"lat":-270}, {"lon":0,"lat":90}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/optimized_route?json={"locations":[{"lon":0,"lat":90}, {"lon":0,"lat":90}], "costing": "NONE","directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/sources_to_targets?json={"sources":[{"lon":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/sources_to_targets?json={"sources":[{"lon":0,"lat":90}],"targets":[{"lon":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        GET,
        R"(/route?json={"locations":[{"lon":0,"lat":0},{"lon":0,"lat":0}],"costing":"pedestrian","avoid_locations":[{"lon":0,"lat":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(
        POST,
        "/trace_attributes",
        R"({"shape":[{"lat":37.8077440,"lon":-122.4197010},{"lat":37.8077440,"lon":-122.4197560},{"lat":37.8077450,"lon":-122.4198180}],"shape_match":"map_snap","best_paths":0,"costing":"pedestrian","directions_options":{"units":"miles", "format":"osrm"}})"),
    http_request_t(
        POST,
        "/trace_attributes",
        R"({"shape":[{"lat":37.8077440,"lon":-122.4197010},{"lat":37.8077440,"lon":-122.4197560},{"lat":37.8077450,"lon":-122.4198180}],"shape_match":"map_snap","best_paths":5,"costing":"pedestrian","directions_options":{"units":"miles", "format":"osrm"}})"),
    http_request_t(POST, "/trace_attributes", R"({"encoded_polyline":
        "mx{ilAdxcupCdJm@v|@rG|n@dEz_AlUng@fMnDlAt}@zTdmAtZvx@`Rr_@~IlUnI`HtDjVnSdOhW|On^|JvXl^dmApGzUjGfYzAtOT~SUdYsFtmAmK~zBkAh`ArAdd@vDng@dEb\\nHvb@bQpp@~IjVbj@ngAjV`q@bL~g@nDjVpVbnBdAfCpeA`yL~CpRnCn]`C~g@l@zUGfx@m@x_AgCxiBe@xl@e@re@yBviCeAvkAe@vaBzArd@jFhb@|ZzgBjEjVzFtZxC`RlEdYz@~I~DxWtTxtA`Gn]fEjV~BzV^dDpBfY\\dZ?fNgDx~BrA~q@xB|^fIp{@lK~|@|T`oBbF|h@re@d_E|EtYvMrdAvCzUxMhaAnStwAnNls@xLjj@tlBr{HxQlt@lEr[jB`\\Gvl@oNjrCaCvm@|@vb@rAl_@~B|]pHvx@j`@lzC|Ez_@~Htn@|DrFzPlhAzFn^zApp@xGziA","shape_match":"map_snap","best_paths":3,"costing":"auto","directions_options":{"units":"miles","format":"osrm"}})"),
};

const std::vector<std::pair<uint16_t, std::string>> osrm_responses{
    {200, "{}"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"DistanceExceeded","message":"Path distance exceeds the max distance limit."})"},
    {400, R"({"code":"DistanceExceeded","message":"Path distance exceeds the max distance limit."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {400,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"}};

boost::property_tree::ptree make_config(const std::vector<std::string>& whitelist = {
                                            "locate",
                                            "route",
                                            "height",
                                            "sources_to_targets",
                                            "optimized_route",
                                            "isochrone",
                                            "trace_route",
                                            "trace_attributes",
                                            "transit_available",
                                            "expansion",
                                            "centroid",
                                            "status",
                                        }) {
  auto run_dir = VALHALLA_BUILD_DIR "test" + std::string(1, filesystem::path::preferred_separator) +
                 "loki_service_tmp";
  if (!filesystem::is_directory(run_dir) && !filesystem::create_directories(run_dir))
    throw std::runtime_error("Couldnt make directory to run from");

  auto config = test::make_config(run_dir,
                                  {
                                      {"service_limits.skadi.max_shape", "100"},
                                      {"service_limits.max_avoid_locations", "0"},
                                  },
                                  {"loki.actions"});

  boost::property_tree::ptree actions;
  for (const auto& action_name : whitelist) {
    boost::property_tree::ptree action;
    action.put("", action_name);
    actions.push_back(std::make_pair("", action));
  }
  config.add_child("loki.actions", actions);

  return config;
}

// config for permanently running server
auto const config = make_config();

zmq::context_t context;
void start_service() {
  // server
  std::thread server(std::bind(&http_server_t::serve,
                               http_server_t(context, config.get<std::string>("httpd.service.listen"),
                                             config.get<std::string>("loki.service.proxy") + "_in",
                                             config.get<std::string>("httpd.service.loopback"),
                                             config.get<std::string>("httpd.service.interrupt"))));
  server.detach();

  // load balancer
  std::thread proxy(std::bind(&proxy_t::forward,
                              proxy_t(context, config.get<std::string>("loki.service.proxy") + "_in",
                                      config.get<std::string>("loki.service.proxy") + "_out")));
  proxy.detach();

  // service worker
  std::thread worker(valhalla::loki::run_service, config);
  worker.detach();
}

void run_requests(const std::vector<http_request_t>& requests,
                  const std::vector<std::pair<uint16_t, std::string>>& responses) {

  // client makes requests and gets back responses in a batch fashion
  auto request = requests.cbegin();
  std::string request_str;
  int success_count = 0;
  http_client_t
      client(context, config.get<std::string>("httpd.service.listen"),
             [&requests, &request, &request_str]() {
               // we dont have any more requests so bail
               if (request == requests.cend()) {
                 return std::make_pair<const void*, size_t>(nullptr, 0);
               }
               // get the string of bytes to send formatted for http protocol
               request_str = request->to_string();
               // LOG_INFO("Loki Test Request :: " + request_str + '\n');
               ++request;
               return std::make_pair<const void*, size_t>(request_str.c_str(), request_str.size());
             },
             [&requests, &request, &responses, &success_count](const void* data, size_t size) {
               auto response = http_response_t::from_string(static_cast<const char*>(data), size);
               EXPECT_EQ(response.code, responses[request - requests.cbegin() - 1].first);

               // Parse as rapidjson::Document which correctly doesn't care about order-dependence
               // of the json-data compared to the boost::property_tree::ptree
               rapidjson::Document response_json, expected_json;
               response_json.Parse(response.body);
               expected_json.Parse(responses[request - requests.cbegin() - 1].second);
               EXPECT_EQ(response_json, expected_json)
                   << "\nExpected Response: " + responses[request - requests.cbegin() - 1].second +
                          "\n, Actual Response: " + response.body;

               ++success_count;
               return request != requests.cend();
             },
             1);
  // request and receive
  client.batch();

  // Make sure that all requests are tested
  EXPECT_EQ(success_count, requests.size());
}

TEST(LokiService, test_failure_requests) {
  run_requests(valhalla_requests, valhalla_responses);
}

TEST(LokiService, test_osrm_failure_requests) {
  run_requests(osrm_requests, osrm_responses);
}

TEST(LokiService, test_actions_whitelist) {
  http_request_info_t info{};

  // check that you only get in if your on the configured list
  for (auto action = Options::Action_MIN; action < Options::Action_ARRAYSIZE;
       action = static_cast<Options::Action>(static_cast<int>(action) + 1)) {

    auto wrong_action = static_cast<Options::Action>(static_cast<int>(action) +
                                                     (action == Options::Action_MAX ? -1 : 1));
    auto cfg = make_config({Options_Action_Enum_Name(wrong_action)});
    loki::loki_worker_t worker(cfg);
    http_request_t request(method_t::GET, "/" + Options_Action_Enum_Name(action));
    auto req_str = request.to_string();
    auto msg = zmq::message_t{reinterpret_cast<void*>(&req_str.front()), req_str.size(),
                              [](void*, void*) {}};
    auto result = worker.work({msg}, reinterpret_cast<void*>(&info), []() {});

    // failed to find that action in the whitelist
    EXPECT_TRUE(result.messages.front().find("Try any") != std::string::npos);

    http_request_t request1(method_t::GET, "/" + Options_Action_Enum_Name(wrong_action));
    req_str = request1.to_string();
    msg = zmq::message_t{reinterpret_cast<void*>(&req_str.front()), req_str.size(),
                         [](void*, void*) {}};
    result = worker.work({msg}, reinterpret_cast<void*>(&info), []() {});

    // found the action this time but failed for no locations
    EXPECT_TRUE(result.messages.front().find("Try any") == std::string::npos);
  }
}

} // namespace

class LokiServiceEnv : public ::testing::Environment {
public:
  void SetUp() override {
    start_service();
  }
};

// Elevation service
int main(int argc, char* argv[]) {
  // make this whole thing bail if it doesnt finish fast
  alarm(180);

  testing::AddGlobalTestEnvironment(new LokiServiceEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
