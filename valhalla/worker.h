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

// TODO: this will go away and Options will be the request object
void ParseApi(const std::string& json_request, Options::Action action, Api& api);
#ifdef HAVE_HTTP
void ParseApi(const prime_server::http_request_t& http_request, Api& api);
#endif

std::string jsonify_error(const valhalla_exception_t& exception, Api& options);
#ifdef HAVE_HTTP
prime_server::worker_t::result_t jsonify_error(const valhalla_exception_t& exception,
                                               prime_server::http_request_info_t& request_info,
                                               Api& options);
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
