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

#include <boost/property_tree/ptree.hpp>

namespace valhalla {

/**
 * Project specific error messages and codes that can be converted to http responses
 */
struct valhalla_exception_t : public std::runtime_error {
  /**
   * Constructs the exception by looking up predefined ones by their codes. If unsuccessful the code
   * will be 0
   * @param code   the code to look up
   * @param extra  an extra string to append to the codes existing method
   */
  valhalla_exception_t(unsigned code, const std::string& extra = "");
  valhalla_exception_t(unsigned code,
                       const std::string& message,
                       unsigned http_code,
                       const std::string& http_message,
                       const std::string& osrm_error,
                       const std::string& statsd_key = "")
      : std::runtime_error(""), code(code), message(message), http_code(http_code),
        http_message(http_message), osrm_error(osrm_error), statsd_key(statsd_key) {
  }
  const char* what() const noexcept override {
    return message.c_str();
  }
  unsigned code;
  std::string message;
  unsigned http_code;
  std::string http_message;
  std::string osrm_error;
  std::string statsd_key;
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
#ifdef HAVE_HTTP
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

// function to add warnings to proto info object
void add_warning(valhalla::Api& api, unsigned code);

#ifdef HAVE_HTTP
prime_server::worker_t::result_t serialize_error(const valhalla_exception_t& exception,
                                                 prime_server::http_request_info_t& request_info,
                                                 Api& options);
namespace worker {
using content_type = prime_server::headers_t::value_type;
const content_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const content_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
const content_type PBF_MIME{"Content-type", "application/x-protobuf"};
const content_type GPX_MIME{"Content-type", "application/gpx+xml;charset=utf-8"};
} // namespace worker

prime_server::worker_t::result_t to_response(const std::string& data,
                                             prime_server::http_request_info_t& request_info,
                                             const Api& options);
#endif

struct statsd_client_t;
class service_worker_t {
public:
  service_worker_t(const boost::property_tree::ptree& config);

  virtual ~service_worker_t();

#ifdef HAVE_HTTP
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

#endif //__VALHALLA_SERVICE_H__
