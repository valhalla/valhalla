#ifndef VALHALLA_TYR_NAVIGATOR_H_
#define VALHALLA_TYR_NAVIGATOR_H_

#include <vector>
#include <string>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/location.h>
#include <valhalla/proto/route.pb.h>

namespace valhalla {
namespace tyr {

const bool kLimitByConseuctiveCount = true;
constexpr uint32_t kElementMaxCount = 4;
constexpr uint32_t kVerbalAlertElementMaxCount = 1;
constexpr uint32_t kVerbalPreElementMaxCount = 2;
constexpr uint32_t kVerbalPostElementMaxCount = 2;
const std::string kVerbalDelim = ", ";

class Navigator {
 public:

  Navigator(const std::string& route_json_str);

  virtual ~Navigator() = default;

  Navigator(Navigator&&) = default;
  Navigator& operator=(Navigator&&) = default;

  Navigator(const Navigator&) = default;
  Navigator& operator=(const Navigator&) = default;

  const valhalla::Route& route() const;

  void OnLocationChanged(const baldr::Location& location);

 protected:

  void SnapToRoute();

  /////////////////////////////////////////////////////////////////////////////
  valhalla::Route route_;

};


}
}

#endif  // VALHALLA_TYR_NAVIGATOR_H_
