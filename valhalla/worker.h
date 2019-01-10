#ifndef __VALHALLA_SERVICE_H__
#define __VALHALLA_SERVICE_H__
#include <string>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/valhalla.h>

#ifdef HAVE_HTTP
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
using namespace prime_server;
#endif

namespace valhalla {

// to use protobuflite we cant use descriptors which means we cant translate enums to strings
// and so we reimplement the ones we use here
namespace odin {
bool DirectionsOptions_Action_Parse(const std::string& action, odin::DirectionsOptions::Action* a);
const std::string& DirectionsOptions_Action_Name(const odin::DirectionsOptions::Action action);
bool Costing_Parse(const std::string& costing, odin::Costing* c);
const std::string& Costing_Name(const odin::Costing costing);
bool ShapeMatch_Parse(const std::string& match, odin::ShapeMatch* s);
const std::string& ShapeMatch_Name(const odin::ShapeMatch match);
bool DirectionsOptions_Format_Parse(const std::string& format, odin::DirectionsOptions::Format* f);
const std::string& DirectionsOptions_Format_Name(const odin::DirectionsOptions::Format match);
const std::string& DirectionsOptions_Units_Name(const odin::DirectionsOptions::Units unit);
bool FilterAction_Parse(const std::string& action, odin::FilterAction* a);
const std::string& FilterAction_Name(const odin::FilterAction action);
bool DirectionsType_Parse(const std::string& dtype, odin::DirectionsType* t);
} // namespace odin

const std::unordered_map<unsigned, std::string>
    error_codes{// loki project 1xx
                {100, "Failed to parse json request"},
                {101, "Try a POST or GET request instead"},
                {106, "Try any of"},
                {107, "Not Implemented"},

                {110, "Insufficiently specified required parameter 'locations'"},
                {111, "Insufficiently specified required parameter 'time'"},
                {112,
                 "Insufficiently specified required parameter 'locations' or 'sources & targets'"},
                {113, "Insufficiently specified required parameter 'contours'"},
                {114, "Insufficiently specified required parameter 'shape' or 'encoded_polyline'"},

                {120, "Insufficient number of locations provided"},
                {121, "Insufficient number of sources provided"},
                {122, "Insufficient number of targets provided"},
                {123, "Insufficient shape provided"},
                {124, "No edge/node costing provided"},
                {125, "No costing method found"},
                {126, "No shape provided"},

                {130, "Failed to parse location"},
                {131, "Failed to parse source"},
                {132, "Failed to parse target"},
                {133, "Failed to parse avoid"},
                {134, "Failed to parse shape"},
                {135, "Failed to parse trace"},
                {136, "durations size not compatible with trace size"},

                {140, "Action does not support multimodal costing"},
                {141, "Arrive by for multimodal not implemented yet"},
                {142, "Arrive by not implemented for isochrones"},

                {150, "Exceeded max locations"},
                {151, "Exceeded max time"},
                {152, "Exceeded max contours"},
                {153, "Too many shape points"},
                {154, "Path distance exceeds the max distance limit"},
                {155,
                 "Outside the valid walking distance at the beginning or end of a multimodal route"},
                {156, "Outside the valid walking distance between stops of a multimodal route"},
                {157, "Exceeded max avoid locations"},
                {158, "Input trace option is out of bounds"},
                {159, "use_timestamps set with no timestamps present"},

                {160, "Date and time required for origin for date_type of depart at"},
                {161, "Date and time required for destination for date_type of arrive by"},
                {162, "Date and time is invalid.  Format is YYYY-MM-DDTHH:MM"},
                {163, "Invalid date_type"},
                {164, "Invalid shape format"},

                {170, "Locations are in unconnected regions. Go check/edit the map at osm.org"},
                {171, "No suitable edges near location"},

                {199, "Unknown"},

                // odin project 2xx
                {200, "Failed to parse intermediate request format"},
                {201, "Failed to parse TripPath"},
                {202, "Could not build directions for TripPath"},

                {210, "Trip path does not have any nodes"},
                {211, "Trip path has only one node"},
                {212, "Trip must have at least 2 locations"},
                {213, "Error - No shape or invalid node count"},

                {220, "Turn degree out of range for cardinal direction"},

                {230, "Invalid TripDirections_Maneuver_Type in method FormTurnInstruction"},
                {231, "Invalid TripDirections_Maneuver_Type in method FormRelativeTwoDirection"},
                {232, "Invalid TripDirections_Maneuver_Type in method FormRelativeThreeDirection"},

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
                {501, "Failed to parse TripDirections"},
                {502, "Maneuver index not found for specified shape index"},

                {599, "Unknown"}};

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

// TODO: this will go away and DirectionsOptions will be the request object
struct valhalla_request_t {
  rapidjson::Document document;
  odin::DirectionsOptions options;

  valhalla_request_t();
  void parse(const std::string& request, odin::DirectionsOptions::Action action);
  void parse(const std::string& request, const std::string& serialized_options);
#ifdef HAVE_HTTP
  void parse(const http_request_t& request);
#endif
};

#ifdef HAVE_HTTP
worker_t::result_t jsonify_error(const valhalla_exception_t& exception,
                                 http_request_info_t& request_info,
                                 const valhalla_request_t& options);
worker_t::result_t to_response(const baldr::json::ArrayPtr& array,
                               http_request_info_t& request_info,
                               const valhalla_request_t& options);
worker_t::result_t to_response(const baldr::json::MapPtr& map,
                               http_request_info_t& request_info,
                               const valhalla_request_t& options);
worker_t::result_t to_response_json(const std::string& json,
                                    http_request_info_t& request_info,
                                    const valhalla_request_t& options);
worker_t::result_t to_response_xml(const std::string& xml,
                                   http_request_info_t& request_info,
                                   const valhalla_request_t& options);
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
  virtual worker_t::result_t work(const std::list<zmq::message_t>& job,
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
  virtual void set_interrupt(const std::function<void()>& interrupt) final;

protected:
  const std::function<void()>* interrupt;
};
} // namespace valhalla

#endif //__VALHALLA_SERVICE_H__
