#ifndef __VALHALLA_WORKER_H__
#define __VALHALLA_WORKER_H__
#include <valhalla/exceptions.h>
#include <valhalla/midgard/util.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>

#include <boost/property_tree/ptree_fwd.hpp>

#ifdef ENABLE_SERVICES
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
#endif

#include <string>

namespace valhalla {

struct hierarchy_limits_config_t {
  std::vector<HierarchyLimits> max_limits;
  std::vector<HierarchyLimits> default_limits;
};

/**
 * Take the json OR pbf request and parse/validate it. If you pass anything but an empty string
 * for the json request the pbf contents are ignored. If the json request is empty it is assumed
 * the pbf is filled out.
 *
 * @param json_request  A json string in the APIs request format
 * @param action        Which action to perform
 * @param api           The pbf request, this will be modified either with the json provided or, if
 *                      already filled out, it will be validated and the json will be ignored
 */
void ParseApi(const std::string& json_request, Options::Action action, Api& api);

/**
 * Parse hierarchy limits from config. Falls back to default values if none are found at the
 * given path.
 *
 * @param config         the property tree to read from.
 * @param hierarchy      name of the algorithm to parse from the right config section
 * @param uses_dist      if true, also parses values for 'expansion within distance'
 */
hierarchy_limits_config_t
parse_hierarchy_limits_from_config(const boost::property_tree::ptree& config,
                                   const std::string& path,
                                   const bool uses_dist);

/**
 * See if the user supplied custom hierarchy limits and possible override with defaults or clamp
 * to max allowed values.
 *
 * @param hierarchy_limits    user supplied hierarchy limits
 * @param cost                mode costing
 * @param config              the max allowed/default hierarchy limits
 * @param allow_modifications whether modifications are allowed
 *
 * @return true if the user passed hierarchy limits but they needed to be tampered with
 */
bool check_hierarchy_limits(std::vector<HierarchyLimits>& hierarchy_limits,
                            sif::cost_ptr_t& cost,
                            const valhalla::Costing_Options& options,
                            const hierarchy_limits_config_t& config,
                            const bool allow_modifications,
                            const bool use_hierarchy_limits);

/**
 * Appends the first and last point of every user provided cost factor line to the locations loki is
 * about to correlate. Thor needs those correlations to edge walk each line.
 *
 * @param options    the request options
 * @param locations  the locations to be correlated, appended to in place
 *
 * @return the index the appended locations start at
 */
int add_cost_factor_locations(const Options& options,
                              google::protobuf::RepeatedPtrField<valhalla::Location>* locations);

/**
 * Moves the correlations loki produced for the locations appended by add_cost_factor_locations back
 * onto their cost factor lines and drops them from `locations` again.
 *
 * @param options    the request options
 * @param locations  the correlated locations
 * @param offset     the index the appended locations start at
 */
void store_cost_factor_locations(Options& options,
                                 google::protobuf::RepeatedPtrField<valhalla::Location>* locations,
                                 int offset);

/**
 * Resolves user provided cost factor lines to edges by edge walking them, stores the
 * result in the costing options and hands it to costing.
 *
 * @param request             the request object
 * @param mode_costing        costing used to edge walk the lines and to add the resolved edges to
 * @param mode                travel mode used to edge walk the lines
 * @param reader              graph reader
 * @param min_allowed_factor  the smallest factor the config admits
 * @param max_allowed_edges   the max number of edges the config allows
 */
void resolve_cost_factor_edges(Api& request,
                               const sif::mode_costing_t& mode_costing,
                               const sif::TravelMode& mode,
                               baldr::GraphReader& reader,
                               double min_allowed_factor,
                               uint64_t max_allowed_edges);

#ifdef ENABLE_SERVICES
/**
 * Take the json OR pbf request and parse/validate it. If you pass a protobuf mime type in the request
 * it is assumed that the body of the request is protobuf bytes and any json will be ignored. Likewise
 * if no protobuf mime was passed then we assume json is either in the body or the query params.
 *
 * @param http_request  The http request object from which we get the request info
 * @param api           The pbf request, this will be modified either with the json provided or, if
 *                      pbf bytes were passed, they will be deserialized into this object and any json
 *                      will be ignored
 */
void ParseApi(const prime_server::http_request_t& http_request, Api& api);
#endif

std::string serialize_error(const valhalla_exception_t& exception, Api& options);

#ifdef ENABLE_SERVICES
prime_server::worker_t::result_t serialize_error(const valhalla_exception_t& exception,
                                                 prime_server::http_request_info_t& request_info,
                                                 Api& options);
namespace worker {
using content_type = prime_server::headers_t::value_type;
const content_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const content_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
const content_type PBF_MIME{"Content-type", "application/x-protobuf"};
const content_type GPX_MIME{"Content-type", "application/gpx+xml;charset=utf-8"};
const content_type TIFF_MIME("Content-type", "image/tiff");
const content_type MVT_MIME("Content-type", "application/vnd.mapbox-vector-tile");
} // namespace worker

prime_server::worker_t::result_t
to_response(const std::string& data,
            prime_server::http_request_info_t& request_info,
            const Api& options,
            const std::vector<std::pair<std::string, std::string>>& additional_headers = {});
#endif

struct statsd_client_t;
class service_worker_t {
public:
  service_worker_t(const boost::property_tree::ptree& config);

  virtual ~service_worker_t();

#ifdef ENABLE_SERVICES
  /**
   * The main work function that stages in the prime_server will call when responding to requests
   *
   * @param  job           the list of messages from the previous hop in the pipeline, each message
   *                       should be a single deserializable object
   * @param  request_info  the http_request_info object used to communicate with the server about
   *                       the state of the request
   * @param  interrupt     a function that may be called periodically and will throw when processing
   *                       should be interrupted
   * @return result_t      the finished bit of work to be either send back to the client or
   *                       forwarded on to the next pipeline stage
   */
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) = 0;
#endif

  /**
   * After forwarding the completed work on, this is called to reset any internal state, deallocate
   * any memory to stay within limits or purge any staged metrics
   */
  virtual void cleanup();

  /**
   * A function which may or may not be called periodically and show throw if computation is
   * supposed to be halted
   * @param  interrupt    the function to be called which should throw
   */
  virtual void set_interrupt(const std::function<void()>* interrupt);

protected:
  /**
   * This converts each protobuf stat into a string and adds it to the queue of unsent stats
   * @param api  The request tracking object which has the tracked stats stored in it
   */
  void enqueue_statistics(Api& api) const;

  /**
   * Returns name of the service used in statistics
   */
  virtual std::string service_name() const = 0;

  /**
   * Used to measure the time it takes to do an action in the current stage of the pipeline.
   * This should be called at the top of the scope in each major action of each worker
   *
   * @param api  The request object where we store the timing information
   * @return an object whose destructor records the elapsed time since construction as a stat
   */
  midgard::Finally<std::function<void()>> measure_scope_time(Api& api) const;

  /**
   * Signals the start of the worker, sends statsd message if so configured
   */
  void started();

  const std::function<void()>* interrupt;
  std::unique_ptr<statsd_client_t> statsd_client;
};
} // namespace valhalla

#endif //__VALHALLA_WORKER_H__
