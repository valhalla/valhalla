#ifndef VALHALLA_TYR_ACTOR_H_
#define VALHALLA_TYR_ACTOR_H_

#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <unordered_map>

namespace valhalla {
namespace tyr {

class actor_t {
public:
  actor_t(const boost::property_tree::ptree& config, bool auto_cleanup = false);
  void cleanup();
  std::string route(const std::string& request_str,
                    const std::function<void()>& interrupt = []() -> void {});
  std::string locate(const std::string& request_str,
                     const std::function<void()>& interrupt = []() -> void {});
  std::string matrix(const std::string& request_str,
                     const std::function<void()>& interrupt = []() -> void {});
  std::string optimized_route(const std::string& request_str,
                              const std::function<void()>& interrupt = []() -> void {});
  std::string isochrone(const std::string& request_str,
                        const std::function<void()>& interrupt = []() -> void {});
  std::string trace_route(const std::string& request_str,
                          const std::function<void()>& interrupt = []() -> void {});
  std::string trace_attributes(const std::string& request_str,
                               const std::function<void()>& interrupt = []() -> void {});
  std::string height(const std::string& request_str,
                     const std::function<void()>& interrupt = []() -> void {});
  std::string transit_available(const std::string& request_str,
                                const std::function<void()>& interrupt = []() -> void {});
  std::string expansion(const std::string& request_str,
                        const std::function<void()>& interrupt = []() -> void {});

protected:
  struct pimpl_t;
  std::shared_ptr<pimpl_t> pimpl;
  bool auto_cleanup;
};

} // namespace tyr
} // namespace valhalla

#endif // VALHALLA_TYR_ACTOR_H_
