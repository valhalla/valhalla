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
#include "odin/worker.h"
#include "thor/worker.h"

using namespace valhalla;
using namespace prime_server;

namespace {

const std::vector<http_request_t> valhalla_requests{
    http_request_t(GET, "/status"),
    http_request_t(GET, R"(/status?json={"verbose": true})"),
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
        R"(/route?json={"locations":[{"lon":0,"lat":0},{"lon":0,"lat":0}],"costing":"pedestrian","exclude_locations":[{"lon":0,"lat":0}]})"),
    http_request_t(POST, "/trace_attributes", R"({"encoded_polyline":
        "mx{ilAdxcupCdJm@v|@rG|n@dEz_AlUng@fMnDlAt}@zTdmAtZvx@`Rr_@~IlUnI`HtDjVnSdOhW|On^|JvXl^dmApGzUjGfYzAtOT~SUdYsFtmAmK~zBkAh`ArAdd@vDng@dEb\\nHvb@bQpp@~IjVbj@ngAjV`q@bL~g@nDjVpVbnBdAfCpeA`yL~CpRnCn]`C~g@l@zUGfx@m@x_AgCxiBe@xl@e@re@yBviCeAvkAe@vaBzArd@jFhb@|ZzgBjEjVzFtZxC`RlEdYz@~I~DxWtTxtA`Gn]fEjV~BzV^dDpBfY\\dZ?fNgDx~BrA~q@xB|^fIp{@lK~|@|T`oBbF|h@re@d_E|EtYvMrdAvCzUxMhaAnStwAnNls@xLjj@tlBr{HxQlt@lEr[jB`\\Gvl@oNjrCaCvm@|@vb@rAl_@~B|]pHvx@j`@lzC|Ez_@~Htn@|DrFzPlhAzFn^zApp@xGziA","shape_match":"map_snap","best_paths":3,"costing":"auto","directions_options":{"units":"miles"}})"),
};

const std::vector<std::pair<uint16_t, std::string>> valhalla_responses{
    {200, R"({"version":")" VALHALLA_VERSION R"(","tileset_last_modified":0})"},
    {200,
     R"({"version":")" VALHALLA_VERSION
     R"(","tileset_last_modified":0,"has_tiles":false,"has_admins":false,"has_timezones":false,"has_live_traffic":false,"bbox":{"features":[],"type":"FeatureCollection"}})"},
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
     R"({"error_code":106,"error":"Try any of: '\/locate' '\/route' '\/height' '\/sources_to_targets' '\/optimized_route' '\/isochrone' '\/trace_route' '\/trace_attributes' '\/transit_available' '\/expansion' '\/centroid' '\/status' ","status_code":404,"status":"Not Found"})"},
    {404,
     R"({"error_code":106,"error":"Try any of: '\/locate' '\/route' '\/height' '\/sources_to_targets' '\/optimized_route' '\/isochrone' '\/trace_route' '\/trace_attributes' '\/transit_available' '\/expansion' '\/centroid' '\/status' ","status_code":404,"status":"Not Found"})"},
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
     R"({"error_code":154,"error":"Path distance exceeds the max distance limit: 250000 meters","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":154,"error":"Path distance exceeds the max distance limit: 250000 meters","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":125,"error":"No costing method found: 'yak'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":125,"error":"No costing method found: 'yak'","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":150,"error":"Exceeded max locations: 20","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":150,"error":"Exceeded max locations: 20","status_code":400,"status":"Bad Request"})"},
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
     R"({"error_code":157,"error":"Exceeded max avoid locations: 0","status_code":400,"status":"Bad Request"})"},
    {400,
     R"({"error_code":153,"error":"Too many shape points: (102). The best paths shape limit is 100","status_code":400,"status":"Bad Request"})"}};

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
        R"(/route?json={"locations":[{"lon":0,"lat":0},{"lon":0,"lat":0}],"costing":"pedestrian","exclude_locations":[{"lon":0,"lat":0}],"directions_options":{"format":"osrm"}})"),
    http_request_t(POST, "/trace_attributes", R"({"encoded_polyline":
        "mx{ilAdxcupCdJm@v|@rG|n@dEz_AlUng@fMnDlAt}@zTdmAtZvx@`Rr_@~IlUnI`HtDjVnSdOhW|On^|JvXl^dmApGzUjGfYzAtOT~SUdYsFtmAmK~zBkAh`ArAdd@vDng@dEb\\nHvb@bQpp@~IjVbj@ngAjV`q@bL~g@nDjVpVbnBdAfCpeA`yL~CpRnCn]`C~g@l@zUGfx@m@x_AgCxiBe@xl@e@re@yBviCeAvkAe@vaBzArd@jFhb@|ZzgBjEjVzFtZxC`RlEdYz@~I~DxWtTxtA`Gn]fEjV~BzV^dDpBfY\\dZ?fNgDx~BrA~q@xB|^fIp{@lK~|@|T`oBbF|h@re@d_E|EtYvMrdAvCzUxMhaAnStwAnNls@xLjj@tlBr{HxQlt@lEr[jB`\\Gvl@oNjrCaCvm@|@vb@rAl_@~B|]pHvx@j`@lzC|Ez_@~Htn@|DrFzPlhAzFn^zApp@xGziA","shape_match":"map_snap","best_paths":3,"costing":"auto","directions_options":{"units":"miles","format":"osrm"}})"),
};

const std::vector<std::pair<uint16_t, std::string>> osrm_responses{
    {200, R"({"version":")" VALHALLA_VERSION R"(","tileset_last_modified":0})"},
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
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"}};

// TODO: we should also test error handling in thor and odin but to do that we need to somehow modify
// how loki processes the request so that we can get it to the next stage in the pipeline. we used
// send json between parts of the request processing pipeline where it was easy to test the parts
// individually in a unit test with json directly. now that we use protobuf its a little harder. what
// follows is a previous set of failure scenarios for testing the thor_worker service
std::list<std::pair<std::string, std::string>>
    failure_request_responses{
        /* {"{\"action\":0,\"locations\":[{\"lat\":\"40.743355\",\"lon\":\"-73.998182\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99814\",\"lat\":\"40.74347\"},\"location_index\":\"1\"}}","Insufficient
         number of locations provided"},
         {"{\"action\":-1,\"locations\":[{\"lat\":\"40.751158\",\"lon\":\"-74.000816\"},{\"lat\":\"40.745696\",\"lon\":\"-73.985023\"},{\"lat\":\"40.739193\",\"lon\":\"-73.980732\"},{\"lat\":\"40.73269\",\"lon\":\"-73.98468\"},{\"lat\":\"40.737893\",\"lon\":\"-73.99189\"}
         ],\"costing\":\"auto\",\"units\":\"mi\",\"correlated_0\":{\"edges\":[{\"id\":\"705396615\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"839614343\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"15737782151\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"136667955080\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.00143\",\"lat\":\"40.75145\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"268738197992\",\"dist\":\"0.7749391\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98515\",\"lat\":\"40.7454\"},\"location_index\":\"1\"},\"correlated_2\":{\"edges\":[{\"id\":\"358127204840\",\"dist\":\"0.288961\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98063\",\"lat\":\"40.73945\"},\"location_index\":\"2\"},\"correlated_3\":{\"edges\":[{\"id\":\"120561826280\",\"dist\":\"0.4931332\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98469\",\"lat\":\"40.73269\"},\"location_index\":\"3\"},\"correlated_4\":{\"edges\":[{\"id\":\"36944181736\",\"dist\":\"0.8432447\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99194\",\"lat\":\"40.73779\"},\"location_index\":\"4\"}}","Unknown
         action"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}
         ],\"costing\":\"walk\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No
         costing method found for 'walk'"},
         {"{\"action\":0,\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Insufficiently
         specified required parameter 'locations'"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\"}],\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Failed
         to parse location"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.743355347975395\",\"lon\":\"-73.99818241596222\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_0\":{\"edges\":[{\"id\":\"14024310487527\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"3523968903\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.0033\",\"lat\":\"40.74981\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ]}}","Failed to parse correlated location"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}
         ],\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No
         edge/node costing provided"},
         {"{\"locations\":[{\"lat\":\"44.41416430998939\",\"lon\":\"-99.80682377703488\",\"type\":\"break\",\"date_time\":\"current\"},{\"lat\":\"44.35331432151491\",\"lon\":\"-99.57611088640988\",\"type\":\"break\"}
         ],\"costing\":\"multimodal\",\"date_time\":{\"type\":\"0\"},\"api_key\":\"valhalla-t_16n1c\",\"correlated_0\":{\"edges\":[{\"id\":\"58687475168\",\"dist\":\"0.2245807\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"59090128352\",\"dist\":\"0.7754193\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.80527\",\"lat\":\"44.41417\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"28354268641\",\"dist\":\"0.3254639\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"28756921825\",\"dist\":\"0.6745361\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.57611\",\"lat\":\"44.35286\"},\"location_index\":\"1\"}}","Cannot
         reach destination - too far from a transit stop"},
         */
    };

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
                                  {{"service_limits.skadi.max_shape", "100"},
                                   {"service_limits.max_exclude_locations", "0"}},
                                  {"loki.actions", "mjolnir.tile_extract", "mjolnir.tile_dir"});

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

// for zmq thead communications
zmq::context_t context;

// this macro is convenient for making all the stages of the service pipeline
#define STAGE(stage)                                                                                 \
  {                                                                                                  \
    std::thread proxy(                                                                               \
        std::bind(&proxy_t::forward,                                                                 \
                  proxy_t(context,                                                                   \
                          config.get<std::string>(std::string(#stage) + ".service.proxy") + "_in",   \
                          config.get<std::string>(std::string(#stage) + ".service.proxy") +          \
                              "_out")));                                                             \
    proxy.detach();                                                                                  \
    std::thread worker(valhalla::stage::run_service, config);                                        \
    worker.detach();                                                                                 \
  }

void start_service() {
  // server
  std::thread server(std::bind(&http_server_t::serve,
                               http_server_t(context, config.get<std::string>("httpd.service.listen"),
                                             config.get<std::string>("loki.service.proxy") + "_in",
                                             config.get<std::string>("httpd.service.loopback"),
                                             config.get<std::string>("httpd.service.interrupt"))));
  server.detach();

  // proxies and workers
  STAGE(loki);
  STAGE(thor);
  STAGE(odin);
}

void run_requests(const std::vector<http_request_t>& requests,
                  const std::vector<std::pair<uint16_t, std::string>>& responses) {

  // client makes requests and gets back responses in a batch fashion
  auto request = requests.cbegin();
  std::string request_str;
  int success_count = 0;
  http_client_t client(
      context, config.get<std::string>("httpd.service.listen"),
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

    // expansion needs a body with an "action" member or parsing will fail
    auto get_endpoint = [](Options::Action action) {
      auto endpoint = Options_Action_Enum_Name(action);
      if (action == Options_Action_expansion) {
        endpoint += R"(?json={"action": "isochrone"})";
      }
      return "/" + endpoint;
    };

    auto wrong_action = static_cast<Options::Action>(static_cast<int>(action) +
                                                     (action == Options::Action_MAX ? -1 : 1));
    auto cfg = make_config({Options_Action_Enum_Name(wrong_action)});
    loki::loki_worker_t worker(cfg);
    http_request_t request(method_t::GET, get_endpoint(action));
    auto req_str = request.to_string();
    auto msg = zmq::message_t{reinterpret_cast<void*>(&req_str.front()), req_str.size(),
                              [](void*, void*) {}};
    auto result = worker.work({msg}, reinterpret_cast<void*>(&info), []() {});

    // failed to find that action in the whitelist
    auto front = result.messages.front();
    EXPECT_TRUE(result.messages.front().find("Try any") != std::string::npos);

    http_request_t request1(method_t::GET, get_endpoint(wrong_action));
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
