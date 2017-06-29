#ifndef __VALHALLA_SERVICE_H__
#define __VALHALLA_SERVICE_H__

#include <string>
#include <unordered_map>
#include <functional>

#include <boost/optional.hpp>

#include <valhalla/valhalla.h>
#include <valhalla/exception.h>
#include <valhalla/baldr/json.h>

#ifdef HAVE_HTTP
#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;
#endif

namespace valhalla {
  namespace service {
    enum ACTION_TYPE {ROUTE = 0, VIAROUTE = 1, LOCATE = 2, ONE_TO_MANY = 3, MANY_TO_ONE = 4, MANY_TO_MANY = 5,
                      SOURCES_TO_TARGETS = 6, OPTIMIZED_ROUTE = 7, ISOCHRONE = 8, TRACE_ROUTE = 9, TRACE_ATTRIBUTES = 10, HEIGHT = 11};
  }
}

namespace std {
  template <>
  struct hash<valhalla::service::ACTION_TYPE> {
    std::size_t operator()(const valhalla::service::ACTION_TYPE& a) const{
      return std::hash<int>()(a);
    }
  };
}

namespace valhalla {
  namespace service {

    const std::unordered_map<std::string, ACTION_TYPE> PATH_TO_ACTION{
      {"/route", ROUTE},
      {"/viaroute", VIAROUTE},
      {"/locate", LOCATE},
      {"/one_to_many", ONE_TO_MANY},
      {"/many_to_one", MANY_TO_ONE},
      {"/many_to_many", MANY_TO_MANY},
      {"/sources_to_targets", SOURCES_TO_TARGETS},
      {"/optimized_route", OPTIMIZED_ROUTE},
      {"/isochrone", ISOCHRONE},
      {"/trace_route", TRACE_ROUTE},
      {"/trace_attributes", TRACE_ATTRIBUTES},
      {"/height", HEIGHT}
    };

    const std::unordered_map<ACTION_TYPE, std::string> ACTION_TO_STRING {
      {ROUTE, "route"},
      {VIAROUTE, "viaroute"},
      {LOCATE, "locate"},
      {ONE_TO_MANY, "one_to_many"},
      {MANY_TO_ONE, "many_to_one"},
      {MANY_TO_MANY, "many_to_many"},
      {SOURCES_TO_TARGETS, "sources_to_targets"},
      {OPTIMIZED_ROUTE, "optimized_route"},
      {ISOCHRONE, "isochrone"},
      {TRACE_ROUTE, "trace_route"},
      {TRACE_ATTRIBUTES, "trace_attributes"},
      {HEIGHT, "height"}
    };

#ifdef HAVE_HTTP
    worker_t::result_t jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info, const boost::optional<std::string>& jsonp = boost::none);
    worker_t::result_t to_response(baldr::json::ArrayPtr array, const boost::optional<std::string>& jsonp, http_request_info_t& request_info);
    worker_t::result_t to_response(baldr::json::MapPtr map, const boost::optional<std::string>& jsonp, http_request_info_t& request_info);
#endif

    class service_worker_t {
     public:
      virtual ~service_worker_t(){};

#ifdef HAVE_HTTP
      /**
       * The main work function that stages in the prime_server will call when responding to requests
       *
       * @param  job           the list of messages from the previous hop in the pipeline, each message should be a single deserializable object
       * @param  request_info  the http_request_info object used to communicate with the server about the state of the request
       * @param  interrupt     a function that may be called periodically and will throw when processing should be interrupted
       * @return result_t      the finished bit of work to be either send back to the client or forwarded on to the next pipeline stage
       */
      virtual worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const worker_t::interrupt_function_t& interrupt) = 0;
#endif

      /**
       * After forwarding the completed work on this is called to reset any internal state or reclaim any memory
       */
      virtual void cleanup() = 0;
    };
  }
}

#endif //__VALHALLA_SERVICE_H__
