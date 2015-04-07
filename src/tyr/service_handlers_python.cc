#include "tyr/handler.h"
#include "tyr/route_handler.h"
#include "tyr/custom_route_handler.h"
#include "tyr/locate_handler.h"
#include "tyr/nearest_handler.h"
#include <valhalla/midgard/logging.h>

#include <string>
#include <sstream>
#include <boost/python.hpp>
#include <boost/python/str.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/extract.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace {

  //statically set the config file and configure logging, throw if you never configured
  //configuring multiple times is wasteful/ineffectual but not harmful
  boost::property_tree::ptree configure(const boost::optional<std::string>& config = boost::none) {
    static boost::optional<boost::property_tree::ptree> pt;

    //if we haven't already loaded one
    if(config && !pt) {
      try {
        //parse the config
        boost::property_tree::ptree temp_pt;
        boost::property_tree::read_json(config.get(), temp_pt);
        pt = temp_pt;

        //configure logging
        boost::optional<boost::property_tree::ptree&> logging_subtree = pt->get_child_optional("tyr.logging");
        if(logging_subtree) {
          auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
          valhalla::midgard::logging::Configure(logging_config);
        }
      }
      catch(...) {
        throw std::runtime_error("Failed to load config from: " + config.get());
      }
    }

    //if it turned out no one ever configured us we throw
    if(!pt)
      throw std::runtime_error("The service was not configured");
    return *pt;
  }
  void py_configure(const std::string& config_file) {
    configure(config_file);
  }

  //use to construct handlers without having to pass the config everytime
  template <class handler_t>
  boost::shared_ptr<handler_t> init(const std::string& request) {
    //slip in the config file
    auto config = configure();
    //parse the request json into a ptree
    std::stringstream stream; stream << request;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(stream, pt);
    //construct the object
    return boost::make_shared<handler_t>(config, pt);
  }

}

BOOST_PYTHON_MODULE(tyr_service) {

  boost::python::def("Configure", py_configure);

  boost::python::class_<valhalla::tyr::LocateHandler,
      boost::shared_ptr<valhalla::tyr::LocateHandler>,
      boost::noncopyable>("LocateHandler", boost::python::no_init)
    .def("__init__", boost::python::make_constructor(init<valhalla::tyr::LocateHandler>))
    .def("Action", &valhalla::tyr::LocateHandler::Action)
  ;

  boost::python::class_<valhalla::tyr::NearestHandler,
      boost::shared_ptr<valhalla::tyr::NearestHandler>,
      boost::noncopyable>("NearestHandler", boost::python::no_init)
    .def("__init__", boost::python::make_constructor(init<valhalla::tyr::NearestHandler>))
    .def("Action", &valhalla::tyr::NearestHandler::Action)
  ;

  boost::python::class_<valhalla::tyr::RouteHandler,
      boost::shared_ptr<valhalla::tyr::RouteHandler>,
      boost::noncopyable>("RouteHandler", boost::python::no_init)
    .def("__init__", boost::python::make_constructor(init<valhalla::tyr::RouteHandler>))
    .def("Action", &valhalla::tyr::RouteHandler::Action)
  ;

  boost::python::class_<valhalla::tyr::CustomRouteHandler,
      boost::shared_ptr<valhalla::tyr::CustomRouteHandler>,
      boost::noncopyable>("CustomRouteHandler", boost::python::no_init)
    .def("__init__", boost::python::make_constructor(init<valhalla::tyr::CustomRouteHandler>))
    .def("Action", &valhalla::tyr::CustomRouteHandler::Action)
  ;

}
