#include "exceptions.h"

#include <valhalla/proto/api.pb.h>

#include <unordered_map>

namespace {

// clang-format off
constexpr const char* HTTP_400 = "Bad Request";
constexpr const char* HTTP_404 = "Not Found";
constexpr const char* HTTP_405 = "Method Not Allowed";
constexpr const char* HTTP_500 = "Internal Server Error";
constexpr const char* HTTP_501 = "Not Implemented";
constexpr const char* HTTP_503 = "Service Unavailable";
constexpr const char* OSRM_INVALID_URL = R"({"code":"InvalidUrl","message":"URL string is invalid."})";
constexpr const char* OSRM_INVALID_SERVICE = R"({"code":"InvalidService","message":"Service name is invalid."})";
constexpr const char* OSRM_INVALID_OPTIONS = R"({"code":"InvalidOptions","message":"Options are invalid."})";
constexpr const char* OSRM_INVALID_VALUE = R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})";
constexpr const char* OSRM_NO_ROUTE = R"({"code":"NoRoute","message":"Impossible route between points"})";
constexpr const char* OSRM_NO_SEGMENT = R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})";
constexpr const char* OSRM_SHUTDOWN = R"({"code":"ServiceUnavailable","message":"The service is shutting down."})";
constexpr const char* OSRM_SERVER_ERROR = R"({"code":"InvalidUrl","message":"Failed to serialize route."})";
constexpr const char* OSRM_DISTANCE_EXCEEDED = R"({"code":"DistanceExceeded","message":"Path distance exceeds the max distance limit."})";
constexpr const char* OSRM_PERIMETER_EXCEEDED = R"({"code":"PerimeterExceeded","message":"Perimeter of avoid polygons exceeds the max limit."})";
constexpr const char* OSRM_BREAKAGE_EXCEEDED = R"({"code":"BreakageDistanceExceeded","message":"All coordinates are too far away from each other"})";

const std::unordered_map<unsigned, valhalla::valhalla_exception_t> error_codes{
    {100, {100, "Failed to parse json request", 400, HTTP_400, OSRM_INVALID_URL, "json_parse_failed"}},
    {101, {101, "Try a POST or GET request instead", 405, HTTP_405, OSRM_INVALID_URL, "wrong_http_method"}},
    {102, {102, "The service is shutting down", 503, HTTP_503, OSRM_SHUTDOWN, "shutting_down"}},
    {103, {103, "Failed to parse pbf request", 400, HTTP_400, OSRM_INVALID_URL, "pbf_parse_failed"}},
    {106, {106, "Try any of", 404, HTTP_404, OSRM_INVALID_SERVICE, "wrong_action"}},
    {107, {107, "Not Implemented", 501, HTTP_501, OSRM_INVALID_SERVICE, "empty_action"}},
    {110, {110, "Insufficiently specified required parameter 'locations'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "locations_parse_failed"}},
    {111, {111, "Insufficiently specified required parameter 'time'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "time_parse_failed"}},
    {112, {112, "Insufficiently specified required parameter 'locations' or 'sources & targets'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "matrix_locations_parse_failed"}},
    {113, {113, "Insufficiently specified required parameter 'contours'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "contours_parse_failed"}},
    {114, {114, "Insufficiently specified required parameter 'shape' or 'encoded_polyline'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "shape_parse_failed"}},
    {115, {115, "Insufficiently specified required parameter 'action'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "action_parse_failed"}},
    {120, {120, "Insufficient number of locations provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "not_enough_locations"}},
    {121, {121, "Insufficient number of sources provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "not_enough_sources"}},
    {122, {122, "Insufficient number of targets provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "not_enough_targets"}},
    {123, {123, "Insufficient shape provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "not_enough_shape"}},
    {124, {124, "No edge/node costing provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "costing_required"}},
    {125, {125, "No costing method found", 400, HTTP_400, OSRM_INVALID_OPTIONS, "wrong_costing"}},
    {126, {126, "No shape provided", 400, HTTP_400, OSRM_INVALID_OPTIONS, "shape_required"}},
    {127, {127, "Recostings require a valid costing parameter", 400, HTTP_400, OSRM_INVALID_OPTIONS, "recosting_parse_failed"}},
    {128, {128, "Recostings require a unique 'name' field for each recosting", 400, HTTP_400, OSRM_INVALID_OPTIONS, "no_recosting_duplicate_names"}},
    {130, {130, "Failed to parse location", 400, HTTP_400, OSRM_INVALID_VALUE, "location_parse_failed"}},
    {131, {131, "Failed to parse source", 400, HTTP_400, OSRM_INVALID_VALUE, "source_parse_failed"}},
    {132, {132, "Failed to parse target", 400, HTTP_400, OSRM_INVALID_VALUE, "target_parse_failed"}},
    {133, {133, "Failed to parse avoid", 400, HTTP_400, OSRM_INVALID_VALUE, "avoid_parse_failed"}},
    {134, {134, "Failed to parse shape", 400, HTTP_400, OSRM_INVALID_VALUE, "shape_parse_failed"}},
    {135, {135, "Failed to parse trace", 400, HTTP_400, OSRM_INVALID_VALUE, "trace_parse_failed"}},
    {136, {136, "durations size not compatible with trace size", 400, HTTP_400, OSRM_INVALID_VALUE, "trace_duration_mismatch"}},
    {137, {137, "Failed to parse polygon", 400, HTTP_400, OSRM_INVALID_VALUE, "polygon_parse_failed"}},
    {140, {140, "Action does not support multimodal costing", 400, HTTP_400, OSRM_INVALID_VALUE, "no_multimodal"}},
    {141, {141, "Arrive by for multimodal not implemented yet", 501, HTTP_501, OSRM_INVALID_VALUE, "no_arrive_by_multimodal"}},
    {142, {142, "Arrive by not implemented for isochrones", 501, HTTP_501, OSRM_INVALID_VALUE, "no_arrive_by_isochrones"}},
    {143, {143, "ignore_closures in costing and exclude_closures in search_filter cannot both be specified", 400, HTTP_400, OSRM_INVALID_VALUE, "closures_conflict"}},
    {144, {144, "Action does not support expansion", 400, HTTP_400, OSRM_INVALID_VALUE, "no_action_for_expansion"}},
    {150, {150, "Exceeded max locations", 400, HTTP_400, OSRM_INVALID_VALUE, "too_many_locations"}},
    {151, {151, "Exceeded max time", 400, HTTP_400, OSRM_INVALID_VALUE, "too_large_time"}},
    {152, {152, "Exceeded max contours", 400, HTTP_400, OSRM_INVALID_VALUE, "too_many_contours"}},
    {153, {153, "Too many shape points", 400, HTTP_400, OSRM_INVALID_VALUE, "too_large_shape"}},
    {154, {154, "Path distance exceeds the max distance limit", 400, HTTP_400, OSRM_DISTANCE_EXCEEDED, "too_large_distance"}},
    {155, {155, "Outside the valid walking distance at the beginning or end of a multimodal route", 400, HTTP_400, OSRM_INVALID_URL, "too_large_first_last_walking_distance"}},
    {156, {156, "Outside the valid walking distance between stops of a multimodal route", 400, HTTP_400, OSRM_INVALID_URL, "too_large_in_between_walking_distance"}},
    {157, {157, "Exceeded max avoid locations", 400, HTTP_400, OSRM_INVALID_VALUE, "too_many_avoids"}},
    {158, {158, "Input trace option is out of bounds", 400, HTTP_400, OSRM_INVALID_VALUE, "trace_option_invalid"}},
    {159, {159, "use_timestamps set with no timestamps present", 400, HTTP_400, OSRM_INVALID_VALUE, "missing_timestamps"}},
    {160, {160, "Date and time required for origin for date_type of depart at", 400, HTTP_400, OSRM_INVALID_OPTIONS, "missing_depart_date"}},
    {161, {161, "Date and time required for destination for date_type of arrive by", 400, HTTP_400, OSRM_INVALID_OPTIONS, "missing_arrive_date"}},
    {162, {162, "Date and time is invalid.  Format is YYYY-MM-DDTHH:MM", 400, HTTP_400, OSRM_INVALID_VALUE, "date_parse_failed"}},
    {163, {163, "Invalid date_type", 400, HTTP_400, OSRM_INVALID_VALUE, "wrong_date_type"}},
    {164, {164, "Invalid shape format", 400, HTTP_400, OSRM_INVALID_VALUE, "wrong_shape_format"}},
    {165, {165, "Date and time required for destination for date_type of invariant", 400, HTTP_400, OSRM_INVALID_OPTIONS, "missing_invariant_date"}},
    {167, {167, "Exceeded maximum circumference for exclude_polygons", 400, HTTP_400, OSRM_PERIMETER_EXCEEDED, "too_large_polygon"}},
    {168, {168, "Invalid expansion property type", 400, HTTP_400, OSRM_INVALID_OPTIONS, "invalid_expansion_property"}},
    {170, {170, "Locations are in unconnected regions. Go check/edit the map at osm.org", 400, HTTP_400, OSRM_NO_ROUTE, "impossible_route"}},
    {171, {171, "No suitable edges near location", 400, HTTP_400, OSRM_NO_SEGMENT, "no_edges_near"}},
    {172, {172, "Exceeded breakage distance for all pairs", 400, HTTP_400, OSRM_BREAKAGE_EXCEEDED, "too_large_breakage_distance"}},
    {199, {199, "Unknown", 500, HTTP_500, OSRM_INVALID_URL, "unknown"}},
    {200, {200, "Failed to parse intermediate request format", 500, HTTP_500, OSRM_INVALID_URL, "pbf_parse_failed"}},
    {201, {201, "Failed to parse TripLeg", 500, HTTP_500, OSRM_INVALID_URL, "trip_parse_failed"}},
    {202, {202, "Could not build directions for TripLeg", 500, HTTP_500, OSRM_INVALID_URL, "directions_building_failed"}},
    {203, {203, "The service is shutting down", 503, HTTP_503, OSRM_SHUTDOWN, "shutting_down"}},
    {210, {210, "Trip path does not have any nodes", 400, HTTP_400, OSRM_INVALID_URL, "no_nodes"}},
    {211, {211, "Trip path has only one node", 400, HTTP_400, OSRM_INVALID_URL, "one_node"}},
    {212, {212, "Trip must have at least 2 locations", 400, HTTP_400, OSRM_INVALID_OPTIONS, "not_enough_locations"}},
    {213, {213, "Error - No shape or invalid node count", 400, HTTP_400, OSRM_INVALID_URL, "shape_parse_failed"}},
    {220, {220, "Turn degree out of range for cardinal direction", 400, HTTP_400, OSRM_INVALID_URL, "wrong_turn_degree"}},
    {230, {230, "Invalid DirectionsLeg_Maneuver_Type in method FormTurnInstruction", 400, HTTP_400, OSRM_INVALID_URL, "wrong_maneuver_form_turn"}},
    {231, {231, "Invalid DirectionsLeg_Maneuver_Type in method FormRelativeTwoDirection", 400, HTTP_400, OSRM_INVALID_URL, "wrong_maneuver_form_relative_two"}},
    {232, {232, "Invalid DirectionsLeg_Maneuver_Type in method FormRelativeThreeDirection", 400, HTTP_400, OSRM_INVALID_URL, "wrong_maneuver_form_relative_three"}},
    {299, {299, "Unknown", 500, HTTP_500, OSRM_INVALID_URL, "unknown"}},
    {312, {312, "Insufficiently specified required parameter 'shape' or 'encoded_polyline'", 400, HTTP_400, OSRM_INVALID_OPTIONS, "shape_parse_failed"}},
    {313, {313, "'resample_distance' must be >= ", 400, HTTP_400, OSRM_INVALID_URL, "wrong_resample_distance"}},
    {314, {314, "Too many shape points", 400, HTTP_400, OSRM_INVALID_VALUE, "too_many_shape_points"}},
    {400, {400, "Unknown action", 400, HTTP_400, OSRM_INVALID_SERVICE, "wrong_action"}},
    {401, {401, "Failed to parse intermediate request format", 500, HTTP_500, OSRM_SERVER_ERROR, "options_parse_failed"}},
    {402, {402, "The service is shutting down", 503, HTTP_503, OSRM_SHUTDOWN, "shutting_down"}},
    {420, {420, "Failed to parse correlated location", 400, HTTP_400, OSRM_INVALID_VALUE, "candidate_parse_failed"}},
    {421, {421, "Failed to parse location", 400, HTTP_400, OSRM_INVALID_VALUE, "location_parse_failed"}},
    {422, {422, "Failed to parse source", 400, HTTP_400, OSRM_INVALID_VALUE, "source_parse_failed"}},
    {423, {423, "Failed to parse target", 400, HTTP_400, OSRM_INVALID_VALUE, "target_parse_failed"}},
    {424, {424, "Failed to parse shape", 400, HTTP_400, OSRM_INVALID_VALUE, "shape_parse_failed"}},
    {430, {430, "Exceeded max iterations in CostMatrix::SourceToTarget", 400, HTTP_400, OSRM_INVALID_URL, "too_many_iterations_cost_matrix"}},
    {440, {440, "Cannot reach destination - too far from a transit stop", 400, HTTP_400, OSRM_INVALID_URL, "transit_unreachable"}},
    {441, {441, "Location is unreachable", 400, HTTP_400, OSRM_INVALID_URL, "matrix_element_unreachable"}},
    {442, {442, "No path could be found for input", 400, HTTP_400, OSRM_NO_ROUTE, "no_path"}},
    {443, {443, "Exact route match algorithm failed to find path", 400, HTTP_400, OSRM_NO_SEGMENT, "shape_match_failed"}},
    {444, {444, "Map Match algorithm failed to find path", 400, HTTP_400, OSRM_NO_SEGMENT, "map_match_failed"}},
    {445, {445, "Shape match algorithm specification in api request is incorrect. Please see documentation for valid shape_match input.", 400, HTTP_400, OSRM_INVALID_URL, "wrong_match_type"}},
    {446, {446, "Remote tar file has changed, service is unavailable", 500, HTTP_500, OSRM_SERVER_ERROR, "remote_tar_changed"}},
    {499, {499, "Unknown", 500, HTTP_500, OSRM_INVALID_URL, "unknown"}},
    {503, {503, "Leg count mismatch", 400, HTTP_400, OSRM_INVALID_URL, "wrong_number_of_legs"}},
    {504, {504, "This service does not support GeoTIFF serialization.", 400, HTTP_400, OSRM_INVALID_VALUE, "unknown"}},
    {599, {599, "Unknown serialization error", 400, HTTP_400, OSRM_INVALID_VALUE, "unknown"}},
};

// unordered map for warning pairs
const std::unordered_map<int, std::string> warning_codes = {
  // 1xx is for deprecations
  {100, R"(auto_shorter costing is deprecated, use "shortest" costing option instead)"},
  {101,
    R"(hov costing is deprecated, use "include_hov2" costing option instead)"},
  {102, R"(auto_data_fix is deprecated, use the "ignore_*" costing options instead)"},
  {103, R"(best_paths has been deprecated, use "alternates" instead)"},
  // 2xx is used for ineffective parameters, i.e. we ignore them because of reasons
  {200, R"(path distance exceeds the max distance limit for time-dependent matrix, ignoring date_time)"},
  {201, R"("sources" have date_time set, but "arrive_by" was requested, ignoring date_time)"},
  {202, R"("targets" have date_time set, but "depart_at" was requested, ignoring date_time)"},
  {203, R"("waiting_time" is set on a location of type "via" or "through", ignoring waiting_time)"},
  {204, R"("exclude_polygons" received invalid input, ignoring exclude_polygons)"},
  {205, R"("disable_hierarchy_pruning" exceeded the max distance, ignoring disable_hierarchy_pruning)"},
  {206, R"(CostMatrix does not consider "targets" with "date_time" set, ignoring date_time)"},
  {207, R"(TimeDistanceMatrix does not consider "shape_format", ignoring shape_format)"},
  {208, R"(Hard exclusions are not allowed on this server, ignoring hard excludes)"},
  {209, R"(Customized hierarchy limits are not allowed on this server, using default hierarchy limits)"},
  {210, R"(Provided hierarchy limits exceeded maximum allowed values, using max allowed hierarchy limits)"},
  {211, R"(This action doesn't support requested format, using json instead)"},
  // 3xx is used when costing or location options were specified but we had to change them internally for some reason
  {300, R"(Many:Many CostMatrix was requested, but server only allows 1:Many TimeDistanceMatrix)"},
  {301, R"(1:Many TimeDistanceMatrix was requested, but server only allows Many:Many CostMatrix)"},
  {302, R"("search_filter.level" was specified without a custom "search_cutoff", setting default default cutoff to )"},
  {303, R"("search_cutoff" exceeds maximum allowed value due to "search_filter.level" being specified, clamping cutoff to )"},
  // 4xx is used when we do sneaky important things the user should be aware of
  {400, R"(CostMatrix turned off destination-only on a second pass for connections: )"}
};
// clang-format on

} // namespace

namespace valhalla {
valhalla_exception_t::valhalla_exception_t(unsigned code, const std::string& extra)
    : std::runtime_error("") {
  auto code_itr = error_codes.find(code);
  if (code_itr != error_codes.cend()) {
    *this = code_itr->second;
  }
  if (!extra.empty())
    message += ": " + extra;
}

// function to add warnings to proto info object
void add_warning(valhalla::Api& api, unsigned code, const std::string& extra) {
  auto warning = warning_codes.find(code);
  if (warning != warning_codes.end()) {
    auto* warning_pbf = api.mutable_info()->mutable_warnings()->Add();
    warning_pbf->set_description(warning->second + extra);
    warning_pbf->set_code(warning->first);
  }
}
} // namespace valhalla
