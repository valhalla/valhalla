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

// Off route threshold in meters
constexpr uint32_t kOffRouteThreshold = 50;

class Navigator {
  public:

    Navigator(const std::string& route_json_str);

    virtual ~Navigator() = default;

    Navigator(Navigator&&) = default;
    Navigator& operator=(Navigator&&) = default;

    Navigator(const Navigator&) = default;
    Navigator& operator=(const Navigator&) = default;

    const valhalla::Route& route() const;
    void set_route(const std::string& route_json_str);

    valhalla::NavigationStatus OnLocationChanged(
        const valhalla::FixLocation& fix_location);


  protected:

    void SetUnits();
    bool HasKilometerUnits() const;

    void SetShapeLengthTime();

    bool IsDestinationShapeIndex(size_t idx) const;

    size_t FindManeuverIndex(size_t begin_search_index, size_t shape_index) const;

    void SnapToRoute(const FixLocation& fix_location,
        NavigationStatus& nav_status);

    /////////////////////////////////////////////////////////////////////////////

    // Specified route to navigate
    valhalla::Route route_;

    // Leg index
    size_t leg_index_;

    // Maneuver index
    size_t maneuver_index_;

    // Boolean units
    bool kilometer_units_;

    // Current leg shape
    std::vector<midgard::PointLL> shape_;

    // Current shape index
    size_t current_shape_index_;

    // Remaining leg length
    std::vector<float> remaining_leg_lengths_;

};


}
}

#endif  // VALHALLA_TYR_NAVIGATOR_H_
