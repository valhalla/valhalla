#ifndef VALHALLA_TYR_ACTOR_H_
#define VALHALLA_TYR_ACTOR_H_

#include <boost/property_tree/ptree.hpp>
#include <memory>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>

namespace valhalla {
namespace tyr {

class actor_t {
public:
  /**
   * Constructor
   * @param config         used to configure loki/thor/odin workers and their graphreaders
   * @param auto_cleanup   whether or not to auto clean workers after each action call
   */
  actor_t(const boost::property_tree::ptree& config, bool auto_cleanup = false);

  /**
   * Constructor
   * @param config         used to configure loki/thor/odin workers
   * @param reader         preconstructed graphreader to share with the workers
   * @param auto_cleanup   whether or not to auto clean workers after each action call
   */
  actor_t(const boost::property_tree::ptree& config,
          baldr::GraphReader& reader,
          bool auto_cleanup = false);

  /**
   * Manually clean the underlying workers
   */
  void cleanup();

  /**
   * Perform the action specified in the options. This will both fill out the api object passed in and
   * return the serialized protobuf bytes if protobuf output was requested or json string if json
   * output was requested
   * @param api        object containing the request options and result after the call
   * @param interrupt  allows the underlying computation to be aborted via the functor throwing
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string act(Api& api, const std::function<void()>* interrupt = nullptr);

  /**
   * Perform the route action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string route(const std::string& request_str,
                    const std::function<void()>* interrupt = nullptr,
                    Api* api = nullptr);

  /**
   * Perform the locate action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string locate(const std::string& request_str,
                     const std::function<void()>* interrupt = nullptr,
                     Api* api = nullptr);

  /**
   * Perform the matrix action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string matrix(const std::string& request_str,
                     const std::function<void()>* interrupt = nullptr,
                     Api* api = nullptr);

  /**
   * Perform the optimized_route action and return json or protobuf depending on which was requested.
   * The request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string optimized_route(const std::string& request_str,
                              const std::function<void()>* interrupt = nullptr,
                              Api* api = nullptr);

  /**
   * Perform the isochrone action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string isochrone(const std::string& request_str,
                        const std::function<void()>* interrupt = nullptr,
                        Api* api = nullptr);

  /**
   * Perform the trace_route action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string trace_route(const std::string& request_str,
                          const std::function<void()>* interrupt = nullptr,
                          Api* api = nullptr);

  /**
   * Perform the trace_attributes action and return json or protobuf depending on which was requested.
   * The request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string trace_attributes(const std::string& request_str,
                               const std::function<void()>* interrupt = nullptr,
                               Api* api = nullptr);

  /**
   * Perform the height action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string height(const std::string& request_str,
                     const std::function<void()>* interrupt = nullptr,
                     Api* api = nullptr);

  /**
   * Perform the transit_available action and return json or protobuf depending on which was
   * requested. The request may either be in the form of a json string provided by the request_str
   * parameter or contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string transit_available(const std::string& request_str,
                                const std::function<void()>* interrupt = nullptr,
                                Api* api = nullptr);

  /**
   * Perform the expansion action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string expansion(const std::string& request_str,
                        const std::function<void()>* interrupt = nullptr,
                        Api* api = nullptr);

  /**
   * Perform the centroid action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string centroid(const std::string& request_str,
                       const std::function<void()>* interrupt = nullptr,
                       Api* api = nullptr);

  /**
   * Perform the status action and return json or protobuf depending on which was requested. The
   * request may either be in the form of a json string provided by the request_str parameter or
   * contained in the api parameter as a deserialized protobuf object
   * @param request_str  json string if json input is being used empty otherwise
   * @param interrupt    allows the underlying computation to be aborted via the functor throwing
   * @param api          protobuffer object which can contain the input request via the options object
   *                     and will be filled out as the request is processed
   * @return json or pbf bytes depending on what was specified in the options object
   */
  std::string status(const std::string& request_str,
                     const std::function<void()>* interrupt = nullptr,
                     Api* api = nullptr);

protected:
  struct pimpl_t;
  std::shared_ptr<pimpl_t> pimpl;
  bool auto_cleanup;
};

} // namespace tyr
} // namespace valhalla

#endif // VALHALLA_TYR_ACTOR_H_
