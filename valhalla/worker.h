#ifndef __VALHALLA_SERVICE_H__
#define __VALHALLA_SERVICE_H__
#include <string>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/util.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/valhalla.h>

#ifdef HAVE_HTTP
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
#endif

namespace valhalla {

const std::unordered_map<unsigned, std::string> error_codes{
    // loki project 1xx
    {100, "Failed to parse json request"},
    {101, "Try a POST or GET request instead"},
    {102, "The service is shutting down"},
    {106, "Try any of"},
    {107, "Not Implemented"},

    {110, "Insufficiently specified required parameter 'locations'"},
    {111, "Insufficiently specified required parameter 'time'"},
    {112, "Insufficiently specified required parameter 'locations' or 'sources & targets'"},
    {113, "Insufficiently specified required parameter 'contours'"},
    {114, "Insufficiently specified required parameter 'shape' or 'encoded_polyline'"},

    {120, "Insufficient number of locations provided"},
    {121, "Insufficient number of sources provided"},
    {122, "Insufficient number of targets provided"},
    {123, "Insufficient shape provided"},
    {124, "No edge/node costing provided"},
    {125, "No costing method found"},
    {126, "No shape provided"},
    {127, "Recostings require both name and costing parameters"},

    {130, "Failed to parse location"},
    {131, "Failed to parse source"},
    {132, "Failed to parse target"},
    {133, "Failed to parse avoid"},
    {134, "Failed to parse shape"},
    {135, "Failed to parse trace"},
    {136, "durations size not compatible with trace size"},
    {137, "Failed to parse polygon"},

    {140, "Action does not support multimodal costing"},
    {141, "Arrive by for multimodal not implemented yet"},
    {142, "Arrive by not implemented for isochrones"},
    {143,
     "ignore_closures in costing and exclude_closures in search_filter cannot both be specified"},

    {150, "Exceeded max locations"},
    {151, "Exceeded max time"},
    {152, "Exceeded max contours"},
    {153, "Too many shape points"},
    {154, "Path distance exceeds the max distance limit"},
    {155, "Outside the valid walking distance at the beginning or end of a multimodal route"},
    {156, "Outside the valid walking distance between stops of a multimodal route"},
    {157, "Exceeded max avoid locations"},
    {158, "Input trace option is out of bounds"},
    {159, "use_timestamps set with no timestamps present"},

    {160, "Date and time required for origin for date_type of depart at"},
    {161, "Date and time required for destination for date_type of arrive by"},
    {162, "Date and time is invalid.  Format is YYYY-MM-DDTHH:MM"},
    {163, "Invalid date_type"},
    {164, "Invalid shape format"},
    {165, "Date and time required for destination for date_type of invariant"},
    {166, "Exceeded max distance"},
    {167, "Exceeded maximum circumference for exclude_polygons"},

    {170, "Locations are in unconnected regions. Go check/edit the map at osm.org"},
    {171, "No suitable edges near location"},
    {172, "Exceeded breakage distance for all pairs"},

    {199, "Unknown"},

    // odin project 2xx
    {200, "Failed to parse intermediate request format"},
    {201, "Failed to parse TripLeg"},
    {202, "Could not build directions for TripLeg"},
    {203, "The service is shutting down"},

    {210, "Trip path does not have any nodes"},
    {211, "Trip path has only one node"},
    {212, "Trip must have at least 2 locations"},
    {213, "Error - No shape or invalid node count"},

    {220, "Turn degree out of range for cardinal direction"},

    {230, "Invalid DirectionsLeg_Maneuver_Type in method FormTurnInstruction"},
    {231, "Invalid DirectionsLeg_Maneuver_Type in method FormRelativeTwoDirection"},
    {232, "Invalid DirectionsLeg_Maneuver_Type in method FormRelativeThreeDirection"},

    {299, "Unknown"},

    // skadi project 3xx
    {304, "Try any of"},
    {305, "Not Implemented"},

    {310, "No shape provided"},
    {311, "Insufficient shape provided"},
    {312, "Insufficiently specified required parameter 'shape' or 'encoded_polyline'"},
    {313, "'resample_distance' must be >= "},
    {314, "Too many shape points"},

    {399, "Unknown"},

    // thor project 4xx
    {400, "Unknown action"},
    {401, "Failed to parse intermediate request format"},
    {402, "The service is shutting down"},

    {420, "Failed to parse correlated location"},
    {421, "Failed to parse location"},
    {422, "Failed to parse source"},
    {423, "Failed to parse target"},
    {424, "Failed to parse shape"},

    {430, "Exceeded max iterations in CostMatrix::SourceToTarget"},

    {440, "Cannot reach destination - too far from a transit stop"},
    {441, "Location is unreachable"},
    {442, "No path could be found for input"},
    {443, "Exact route match algorithm failed to find path"},
    {444, "Map Match algorithm failed to find path"},
    {445, "Shape match algorithm specification in api request is incorrect. Please see "
          "documentation for valid shape_match input."},

    {499, "Unknown"},

    // tyr project 5xx
    {500, "Failed to parse intermediate request format"},
    {501, "Failed to parse DirectionsLeg"},
    {502, "Maneuver index not found for specified shape index"},
    {503, "Leg count mismatch"},

    {599, "Unknown"}};

/**
 * Project specific error messages and codes that can be converted to http responses
 */
struct valhalla_exception_t : public std::runtime_error {
  valhalla_exception_t(unsigned code, const boost::optional<std::string>& extra = boost::none);
  const char* what() const noexcept override {
    return message.c_str();
  }
  unsigned code;
  std::string message;
  boost::optional<std::string> extra;
  std::string http_message;
  unsigned http_code;
};

// time this whole method and save that statistic
inline midgard::scoped_timer<> measure_scope_time(Api& api, const std::string& statistic_name) {
  // we take a copy of the stat to avoid cases when its a temporary and goes out of scope before the
  // lambda below goes to actually use it
  return midgard::scoped_timer<>([&api, statistic_name](
                                     const midgard::scoped_timer<>::duration_t& elapsed) {
    auto e = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed).count();
    auto* stat = api.mutable_info()->mutable_statistics()->Add();
    stat->set_name(statistic_name);
    stat->set_value(e);
  });
}

// TODO: this will go away and Options will be the request object
void ParseApi(const std::string& json_request, Options::Action action, Api& api);
#ifdef HAVE_HTTP
void ParseApi(const prime_server::http_request_t& http_request, Api& api);
#endif

std::string jsonify_error(const valhalla_exception_t& exception, const Api& options);
#ifdef HAVE_HTTP
prime_server::worker_t::result_t jsonify_error(const valhalla_exception_t& exception,
                                               prime_server::http_request_info_t& request_info,
                                               const Api& options);
namespace worker {
using content_type = prime_server::headers_t::value_type;
const content_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const content_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
const content_type XML_MIME{"Content-type", "text/xml;charset=utf-8"};
const content_type GPX_MIME{"Content-type", "application/gpx+xml;charset=utf-8"};
} // namespace worker

prime_server::worker_t::result_t
to_response(const std::string& data,
            prime_server::http_request_info_t& request_info,
            const Api& options,
            const worker::content_type& content_type = worker::JSON_MIME,
            const bool as_attachment = false);
#endif

class service_worker_t {
public:
  service_worker_t();

  virtual ~service_worker_t();

#ifdef HAVE_HTTP
  /**
   * The main work function that stages in the prime_server will call when responding to requests
   *
   * @param  job           the list of messages from the previous hop in the pipeline, each message
   * should be a single deserializable object
   * @param  request_info  the http_request_info object used to communicate with the server about
   * the state of the request
   * @param  interrupt     a function that may be called periodically and will throw when processing
   * should be interrupted
   * @return result_t      the finished bit of work to be either send back to the client or
   * forwarded on to the next pipeline stage
   */
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) = 0;
#endif

  /**
   * After forwarding the completed work on this is called to reset any internal state or reclaim
   * any memory
   */
  virtual void cleanup() = 0;

  /**
   * A function which may or may not be called periodically and show throw if computation is
   * supposed to be halted
   * @param  interrupt    the function to be called which should throw
   */
  virtual void set_interrupt(const std::function<void()>* interrupt);

protected:
  const std::function<void()>* interrupt;
};
} // namespace valhalla

#endif //__VALHALLA_SERVICE_H__
