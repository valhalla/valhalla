#ifndef VALHALLA_TYR_ACTOR_H_
#define VALHALLA_TYR_ACTOR_H_

#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <unordered_map>

namespace valhalla {
  namespace tyr {
    enum ACTION_TYPE {ROUTE = 0, VIAROUTE = 1, LOCATE = 2, ONE_TO_MANY = 3, MANY_TO_ONE = 4, MANY_TO_MANY = 5,
      SOURCES_TO_TARGETS = 6, OPTIMIZED_ROUTE = 7, ISOCHRONE = 8, TRACE_ROUTE = 9, TRACE_ATTRIBUTES = 10, HEIGHT = 11,
      TRANSIT_AVAILABLE = 12};
  }
}

namespace std {
  template <>
  struct hash<valhalla::tyr::ACTION_TYPE> {
    inline std::size_t operator()(const valhalla::tyr::ACTION_TYPE& a) const{
      return std::hash<int>()(a);
    }
  };
}

namespace valhalla {
  namespace tyr {

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
      {"/height", HEIGHT},
      {"/transit_available", TRANSIT_AVAILABLE},

      {"route", ROUTE},
      {"viaroute", VIAROUTE},
      {"locate", LOCATE},
      {"one_to_many", ONE_TO_MANY},
      {"many_to_one", MANY_TO_ONE},
      {"many_to_many", MANY_TO_MANY},
      {"sources_to_targets", SOURCES_TO_TARGETS},
      {"optimized_route", OPTIMIZED_ROUTE},
      {"isochrone", ISOCHRONE},
      {"trace_route", TRACE_ROUTE},
      {"trace_attributes", TRACE_ATTRIBUTES},
      {"height", HEIGHT},
      {"transit_available", TRANSIT_AVAILABLE}
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
      {HEIGHT, "height"},
      {TRANSIT_AVAILABLE, "transit_available"}
    };

    class actor_t {
     public:
      actor_t(const boost::property_tree::ptree& config, bool auto_cleanup = false);
      void cleanup();
      std::string route(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string locate(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string matrix(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string optimized_route(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string isochrone(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string trace_route(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string trace_attributes(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string height(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
      std::string transit_available(const std::string& request_str, const std::function<void ()>& interrupt = []()->void{});
     protected:
      struct pimpl_t;
      std::shared_ptr<pimpl_t> pimpl;
      bool auto_cleanup;
    };

  }
}


#endif //VALHALLA_TYR_ACTOR_H_
