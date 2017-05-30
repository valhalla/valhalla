#ifndef VALHALLA_TYR_NAVIGATOR_H_
#define VALHALLA_TYR_NAVIGATOR_H_

#include <vector>
#include <string>
#include <tuple>

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

// Close to origin threshold in meters
constexpr uint32_t kOnRouteCloseToOriginThreshold = 20;

// Closest point tuple indexes
constexpr size_t kClosestPoint = 0;
constexpr size_t kClosestPointDistance = 1;
constexpr size_t kClosestPointSegmentIndex = 2;

// Used instruction tuple indexes
constexpr size_t kTransitionAlert = 0;
constexpr size_t kPreTransition = 1;
constexpr size_t kPostTransition = 2;

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

    void SetUsedInstructions();

    bool IsDestinationShapeIndex(size_t idx) const;

    size_t FindManeuverIndex(size_t begin_search_index, size_t shape_index) const;
    size_t RfindManeuverIndex(size_t rbegin_search_index, size_t shape_index) const;

    void SnapToRoute(const FixLocation& fix_location,
        NavigationStatus& nav_status);

    bool StartingNavigation(const NavigationStatus_RouteState& prev_route_state,
        const NavigationStatus_RouteState& curr_route_state) const;

    bool OnRouteLocationCloseToOrigin(const NavigationStatus& nav_status) const;

    float UnitsToMeters(float units) const;

    size_t GetWordCount(const std::string& instruction) const;

    /////////////////////////////////////////////////////////////////////////////

    // Specified route to navigate
    valhalla::Route route_;

    // Current route state of navigator
    NavigationStatus_RouteState route_state_;

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

    // Maneuver speeds in units per second
    std::vector<float> maneuver_speeds_;

    // Remaining leg length and time
    std::vector<std::pair<float, uint32_t>> remaining_leg_values_;

    // List of tuples by maneuver index that keeps track of the used instructions
    std::vector<std::tuple<bool, bool, bool>> used_instructions_;

};


}
}

#endif  // VALHALLA_TYR_NAVIGATOR_H_
