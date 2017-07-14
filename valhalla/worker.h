#ifndef __VALHALLA_SERVICE_H__
#define __VALHALLA_SERVICE_H__

#include <string>
#include <unordered_map>
#include <functional>

#include <boost/optional.hpp>

#include <valhalla/valhalla.h>
#include <valhalla/exception.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/rapidjson_utils.h>

#ifdef HAVE_HTTP
#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;
#endif

namespace valhalla {

#ifdef HAVE_HTTP
  rapidjson::Document from_request(const http_request_t& request);
  worker_t::result_t jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info, const boost::optional<std::string>& jsonp = boost::none);
  worker_t::result_t to_response(baldr::json::ArrayPtr array, const boost::optional<std::string>& jsonp, http_request_info_t& request_info);
  worker_t::result_t to_response(baldr::json::MapPtr map, const boost::optional<std::string>& jsonp, http_request_info_t& request_info);
#endif

  class service_worker_t {
   public:
    service_worker_t();

    virtual ~service_worker_t();

#ifdef HAVE_HTTP
    /**
     * The main work function that stages in the prime_server will call when responding to requests
     *
     * @param  job           the list of messages from the previous hop in the pipeline, each message should be a single deserializable object
     * @param  request_info  the http_request_info object used to communicate with the server about the state of the request
     * @param  interrupt     a function that may be called periodically and will throw when processing should be interrupted
     * @return result_t      the finished bit of work to be either send back to the client or forwarded on to the next pipeline stage
     */
    virtual worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt) = 0;
#endif

    /**
     * After forwarding the completed work on this is called to reset any internal state or reclaim any memory
     */
    virtual void cleanup() = 0;

    /**
     * A function which may or may not be called periodically and show throw if computation is supposed to be halted
     * @param  interrupt    the function to be called which should throw
     */
    virtual void set_interrupt(const std::function<void ()>& interrupt) final;

   protected:
    const std::function<void ()>* interrupt;

  };
}

#endif //__VALHALLA_SERVICE_H__
