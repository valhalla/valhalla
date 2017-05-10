#ifndef VALHALLA_TYR_NAVIGATOR_H_
#define VALHALLA_TYR_NAVIGATOR_H_

#include <vector>
#include <string>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/route.pb.h>
#include <valhalla/proto/navigator.pb.h>

namespace valhalla {
namespace tyr {

class Navigator {
  public:

    Navigator(const std::string& route_json_str);

    virtual ~Navigator() = default;

    Navigator(Navigator&&) = default;
    Navigator& operator=(Navigator&&) = default;

    Navigator(const Navigator&) = default;
    Navigator& operator=(const Navigator&) = default;

    const valhalla::Route& route() const;

    valhalla::NavigationStatus OnLocationChanged(
        const valhalla::FixLocation& fix_location);

  protected:

    void SnapToRoute(const FixLocation& fix_location,
        NavigationStatus& nav_status);

    /////////////////////////////////////////////////////////////////////////////

    // Specified route to navigate
    valhalla::Route route_;

    // Current leg shape
    std::vector<midgard::PointLL> shape_;

};


}
}

#endif  // VALHALLA_TYR_NAVIGATOR_H_
