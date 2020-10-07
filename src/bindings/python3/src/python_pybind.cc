#include <pybind11/pybind11.h>

#include "baldr/rapidjson_utils.h"
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"


namespace {

// statically set the config file and configure logging, throw if you never configured
// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree&
configure(const boost::optional<std::string>& config = boost::none) {
  static boost::optional<boost::property_tree::ptree> pt;
  // if we haven't already loaded one
  if (config && !pt) {
    try {
      // parse the config
      boost::property_tree::ptree temp_pt;
      rapidjson::read_json(config.get(), temp_pt);
      pt = temp_pt;

      // configure logging
      boost::optional<boost::property_tree::ptree&> logging_subtree =
          pt->get_child_optional("tyr.logging");
      if (logging_subtree) {
        auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                       std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
        valhalla::midgard::logging::Configure(logging_config);

        for (auto const& pair: logging_config) {
            std::cout << pair.first << pair.second << "}\n";
        }
      }
    } catch (...) { throw std::runtime_error("Failed to load config from: " + config.get()); }
  }

  // if it turned out no one ever configured us we throw
  if (!pt) {
    throw std::runtime_error("The service was not configured");
  }
  return *pt;
}
void py_configure(const std::string& config_file) {
  configure(config_file);
}
} // namespace


namespace py = pybind11;
//using namespace pybind11::literals

PYBIND11_MODULE(example, m)
{
    m.def("Configure", py_configure);

    py::class_<valhalla::tyr::actor_t, std::shared_ptr<valhalla::tyr::actor_t>>(m, "Actor")
        .def(py::init<>([](){
            return std::make_shared<valhalla::tyr::actor_t>(configure(), true);
        }))
        //.def("Route", (std::string (valhalla::tyr::actor_t::*)(std::string)) &valhalla::tyr::actor_t,  py::arg("request_str"))
        //.def("Route", py::overload_cast<std::string&>(&py::self::route));
        .def("Route", py::overload_cast<std::string&>(new valhalla::tyr::actor_t(configure(), true)));
        //.def("Route", &valhalla::tyr::actor_t::route, "Doc", py::arg("request_str"), py::arg("interrupt"), py::arg("api"));
        //.def("Route", &valhalla::tyr::actor_t::route, "Doc", py::arg("request_str"));
        //.def("Route", (std::string (valhalla::tyr::actor_t&, std::string&)) &valhalla::tyr::actor_t::route);
}