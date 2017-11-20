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

// Pre-transition base threshold in seconds
constexpr uint32_t kPreTransitionBaseThreshold = 4;

// Transition alert and pre-transition time delta in seconds
constexpr uint32_t kAlertPreTimeDelta = 2;

// Number of words per second - used to calculate pre-transition threshold
constexpr float kWordsPerSecond = 2.5f;

// Minimum speed threshold in meters per second (3.6 KPH or ~2.2 MPH)
constexpr float kMinSpeedThreshold = 1.f;

// Minimum speed in meters per second for certain transition alert types
constexpr uint32_t kInitialLongTransitionAlertMinSpeed = 28; // ~62.6 MPH
constexpr uint32_t kInitialShortTransitionAlertMinSpeed = 20; // ~44.7 MPH
constexpr uint32_t kFinalLongTransitionAlertMinSpeed = 28; // ~62.6 MPH
constexpr uint32_t kFinalMediumTransitionAlertMinSpeed = 10; // ~22.4 MPH

// Post-transition lower and upper bounds in seconds
constexpr uint32_t kPostTransitionLowerBound = 2;
constexpr uint32_t kPostTransitionUpperBound = 6;

// Closest point tuple indexes
constexpr size_t kClosestPoint = 0;
constexpr size_t kClosestPointDistance = 1;
constexpr size_t kClosestPointSegmentIndex = 2;

// Used instruction tuple indexes
constexpr size_t kInitialTransitionAlert = 0; // 2 or 1 miles depending on speed
constexpr size_t kFinalTransitionAlert = 1;   // half or quarter mile depending on speed
constexpr size_t kPreTransition = 2;
constexpr size_t kPostTransition = 3;

///////////////////////////////////////////////////////////////////////////////
// Metric values for transition alert processing

// Transition alert upper and lower deltas
constexpr float kTransitionAlertLowerMetricDelta = 0.05f; // 50 meters
constexpr float kTransitionAlertUpperMetricDelta = 0.05f; // 50 meters

// Initial long transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kInitialLongTransitionAlertMetricLength = 3.0f; // three kilometers
constexpr float kInitialLongTransitionAlertLowerMetricLength = kInitialLongTransitionAlertMetricLength - kTransitionAlertLowerMetricDelta;
constexpr float kInitialLongTransitionAlertUpperMetricLength = kInitialLongTransitionAlertMetricLength + kTransitionAlertUpperMetricDelta;
constexpr float kInitialLongTransitionAlertMinManeuverMetricLength = kInitialLongTransitionAlertMetricLength * 2.0f;

// Initial short transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kInitialShortTransitionAlertMetricLength = 1.0f; // one kilometer
constexpr float kInitialShortTransitionAlertUpperMetricLength = kInitialShortTransitionAlertMetricLength + kTransitionAlertUpperMetricDelta;
constexpr float kInitialShortTransitionAlertLowerMetricLength = kInitialShortTransitionAlertMetricLength - kTransitionAlertLowerMetricDelta;
constexpr float kInitialShortTransitionAlertMinManeuverMetricLength = kInitialShortTransitionAlertMetricLength * 2.0f;

// Final long transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kFinalLongTransitionAlertMetricLength = 0.8f; // 800 meters
constexpr float kFinalLongTransitionAlertLowerMetricLength = kFinalLongTransitionAlertMetricLength - kTransitionAlertLowerMetricDelta;
constexpr float kFinalLongTransitionAlertUpperMetricLength = kFinalLongTransitionAlertMetricLength + kTransitionAlertUpperMetricDelta;
constexpr float kFinalLongTransitionAlertMinManeuverMetricLength = kFinalLongTransitionAlertMetricLength * 2.0f;

// Final medium transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kFinalMediumTransitionAlertMetricLength = 0.4f; // 400 meters
constexpr float kFinalMediumTransitionAlertLowerMetricLength = kFinalMediumTransitionAlertMetricLength - kTransitionAlertLowerMetricDelta;
constexpr float kFinalMediumTransitionAlertUpperMetricLength = kFinalMediumTransitionAlertMetricLength + kTransitionAlertUpperMetricDelta;
constexpr float kFinalMediumTransitionAlertMinManeuverMetricLength = kFinalMediumTransitionAlertMetricLength * 2.0f;

// Final short transition alert length, length bounds, and
// minimum maneuver length threshold
// TODO: maybe refactor for just short values less than medium length?
constexpr float kFinalShortTransitionAlertMetricLength = 0.15f; // 150 meters
constexpr float kFinalShortTransitionAlertLowerMetricLength = kFinalShortTransitionAlertMetricLength - (kTransitionAlertLowerMetricDelta * 0.6667f);
constexpr float kFinalShortTransitionAlertUpperMetricLength = kFinalShortTransitionAlertMetricLength + (kTransitionAlertUpperMetricDelta * 0.6667f);


///////////////////////////////////////////////////////////////////////////////
// Imperial values for transition alert processing

// Transition alert upper and lower deltas
constexpr float kTransitionAlertLowerImperialDelta = 0.0310686f; // ~164 feet
constexpr float kTransitionAlertUpperImperialDelta = 0.0310686f; // ~164 feet

// Initial long transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kInitialLongTransitionAlertImperialLength = 2.0f; // two miles
constexpr float kInitialLongTransitionAlertLowerImperialLength = kInitialLongTransitionAlertImperialLength - kTransitionAlertLowerImperialDelta;
constexpr float kInitialLongTransitionAlertUpperImperialLength = kInitialLongTransitionAlertImperialLength + kTransitionAlertUpperImperialDelta;
constexpr float kInitialLongTransitionAlertMinManeuverImperialLength = kInitialLongTransitionAlertImperialLength * 2.0f;

// Initial short transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kInitialShortTransitionAlertImperialLength = 1.0f; // one mile
constexpr float kInitialShortTransitionAlertUpperImperialLength = kInitialShortTransitionAlertImperialLength + kTransitionAlertUpperImperialDelta;
constexpr float kInitialShortTransitionAlertLowerImperialLength = kInitialShortTransitionAlertImperialLength - kTransitionAlertLowerImperialDelta;
constexpr float kInitialShortTransitionAlertMinManeuverImperialLength = kInitialShortTransitionAlertImperialLength * 2.0f;

// Final long transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kFinalLongTransitionAlertImperialLength = 0.5f; // half mile
constexpr float kFinalLongTransitionAlertLowerImperialLength = kFinalLongTransitionAlertImperialLength - kTransitionAlertLowerImperialDelta;
constexpr float kFinalLongTransitionAlertUpperImperialLength = kFinalLongTransitionAlertImperialLength + kTransitionAlertUpperImperialDelta;
constexpr float kFinalLongTransitionAlertMinManeuverImperialLength = kFinalLongTransitionAlertImperialLength * 2.0f;

// Final medium transition alert length, length bounds, and
// minimum maneuver length threshold
constexpr float kFinalMediumTransitionAlertImperialLength = 0.25f; // quarter mile
constexpr float kFinalMediumTransitionAlertLowerImperialLength = kFinalMediumTransitionAlertImperialLength - kTransitionAlertLowerImperialDelta;
constexpr float kFinalMediumTransitionAlertUpperImperialLength = kFinalMediumTransitionAlertImperialLength + kTransitionAlertUpperImperialDelta;
constexpr float kFinalMediumTransitionAlertMinManeuverImperialLength = kFinalMediumTransitionAlertImperialLength * 2.0f;

// Final short transition alert length, length bounds, and
// minimum maneuver length threshold
// TODO: maybe refactor for just short values less than medium length?
constexpr float kFinalShortTransitionAlertImperialLength = 0.095f; // miles = 500 feet
constexpr float kFinalShortTransitionAlertLowerImperialLength = kFinalShortTransitionAlertImperialLength - (kTransitionAlertLowerImperialDelta * 0.6667f);
constexpr float kFinalShortTransitionAlertUpperImperialLength = kFinalShortTransitionAlertImperialLength + (kTransitionAlertUpperImperialDelta * 0.6667f);

///////////////////////////////////////////////////////////////////////////////
class Navigator {
  public:

    Navigator();

    virtual ~Navigator() = default;

    Navigator(Navigator&&) = default;
    Navigator& operator=(Navigator&&) = default;

    Navigator(const Navigator&) = default;
    Navigator& operator=(const Navigator&) = default;

    /**
     * Sets the route path for the navigator to process.
     * Returns a NavigationStatus_RouteState_kInitialized route state
     * if no errors occurred; otherwise, it returns a
     * NavigationStatus_RouteState_kInvalid route state.
     *
     * @param  route_json_str  A string containing a json route response.
     *
     * @return a NavigationStatus_RouteState_kInitialized route state
     * if no errors occurred; otherwise, it returns a
     * NavigationStatus_RouteState_kInvalid route state.
     */
    NavigationStatus SetRoute(const std::string& route_json_str);

    /**
     * Passes in the current fix location of the user. This method will snap
     * the location to the route and verify that the user is still on the route.
     * Also, this method will determine if an instruction needs to be called out
     * for the user.
     *
     * @param  fix_location  The current fix location of user.
     *
     * @return the navigation status depending on the fix location in relation
     * to the route path.
     * TODO: explain route states
     */
    NavigationStatus OnLocationChanged(const FixLocation& fix_location);


  protected:

    /**
     * Assigns the kilometer units boolean based on the value specified
     * in the route.
     */
    void InitializeDistanceUnits();

    /**
     * Returns true if the distance units are in kilometers; otherwise, return false.
     *
     * @return true if the distance units are in kilometers; otherwise, return false.
     */
    bool HasKilometerUnits() const;

    /**
     * Assigns the shape, maneuver speeds, and remaining leg length & time
     * based on the route.
     */
    void InitializeShapeLengthTime();

    /**
     * Initializes the used instruction boolean values for each maneuver and
     * instruction type.
     */
    void InitializeUsedInstructions();

    /**
     * Returns true if the specified index is the route leg destination shape
     * index; otherwise, returns false.
     *
     * @param  idx  The specified shape index to verify.
     *
     * @return true if the specified index is the route leg destination shape
     * index; otherwise, returns false.
     */
    bool IsDestinationShapeIndex(size_t idx) const;

    /**
     * Returns true if the specified index is the route leg start maneuver
     * index; otherwise, returns false.
     *
     * @param  idx  The specified maneuver index to verify.
     *
     * @return true if the specified index is the route leg start maneuver
     * index; otherwise, returns false.
     */
    bool IsStartManeuverIndex(size_t idx) const;

    /**
     * Returns true if the specified index is the route leg destination maneuver
     * index; otherwise, returns false.
     *
     * @param  idx  The specified maneuver index to verify.
     *
     * @return true if the specified index is the route leg destination maneuver
     * index; otherwise, returns false.
     */
    bool IsDestinationManeuverIndex(size_t idx) const;

    /**
     * Returns the maneuver index that contains the specified shape index.
     *
     * @param  begin_search_index  The maneuver index to start the search.
     * @param  shape_index  The target shape index to find the associated maneuver.
     *
     * @return the maneuver index that contains the specified shape index.
     */
    size_t FindManeuverIndex(size_t begin_search_index, size_t shape_index) const;

    /**
     * Returns the maneuver index that contains the specified shape index by
     * searching in reverse from the specified maneuver index.
     *
     * @param  rbegin_search_index  The maneuver index to start the search in reverse.
     * @param  shape_index  The target shape index to find the associated maneuver.
     *
     * @return the maneuver index that contains the specified shape index by
     * searching in reverse from the specified maneuver index.
     */
    size_t RfindManeuverIndex(size_t rbegin_search_index, size_t shape_index) const;

    /**
     * Find the closest point on the route that corresponds to the specified
     * fix location. If a valid snap point is found then the navigation
     * status will be populated and returned with a route state of 'tracking'.
     * Otherwise, a route state of 'invalid' will be returned.
     *
     * @param  fix_location  The current fix location of user.
     */
    NavigationStatus SnapToRoute(const FixLocation& fix_location);

    /**
     * Returns true if navigation is just starting (going from initialized to
     * tracking state); otherwise, returns false.
     *
     * @param  prev_route_state  The previous route state.
     * @param  curr_route_state  The current route state.
     *
     * @return true if navigation is just starting (going from initialized to
     * tracking state); otherwise, returns false.
     */
    bool StartingNavigation(const NavigationStatus_RouteState& prev_route_state,
        const NavigationStatus_RouteState& curr_route_state) const;

    /**
     * Returns true if the snapped location is close to the route leg origin;
     * otherwise, returns false.
     *
     * @param  nav_status  The current navigation status.
     *
     * @return true if the snapped location is close to the route leg origin;
     * otherwise, returns false.
     */
    bool OnRouteLocationCloseToOrigin(const NavigationStatus& nav_status) const;

    /**
     * Transforms the specified unit value to meters.
     *
     * @param  units  The length to transform to meters.
     *
     * @return the meter equivalent of the specified unit value.
     */
    float UnitsToMeters(float units) const;

    /**
     * Returns the number of words in the specified instruction string.
     *
     * @param  instruction  The string to process the word count.
     *
     * @return the number of words in the specified instruction string.
     */
    size_t GetWordCount(const std::string& instruction) const;

    /**
     * Returns the time traveled on the current maneuver.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     *
     * @return the time traveled on the current maneuver.
     */
    uint32_t GetSpentManeuverTime(const FixLocation& fix_location,
        const NavigationStatus& nav_status) const;

    /**
     * Returns the remaining time to complete the current maneuver.
     * If the specified fix location has speed then it will be used to calculate
     * the remaining time; otherwise, the default data speed is used for the
     * calculation.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     *
     * @return the remaining time to complete the current maneuver.
     */
    uint32_t GetRemainingManeuverTime(const FixLocation& fix_location,
        const NavigationStatus& nav_status) const;

    /**
     * Returns the time in seconds prior to the transition point when a
     * pre-transition instruction should be announced. The number of words
     * in the instruction is used when determining the threshold.
     *
     * @param  instruction_index  The instruction index to process.
     *
     * @return the time in seconds prior to the transition point when a
     * pre-transition instruction should be announced.
     */
    uint32_t GetPreTransitionThreshold(size_t instruction_index) const;

    /**
     * Returns true if the current transition alert is close to the
     * pre-transition; otherwise, return false.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     * @param  instruction_index  The instruction index to process.
     *
     * @return true if the current transition alert is close to the
     * pre-transition; otherwise, return false.
     */
    bool IsAlertCloseToPre(const FixLocation& fix_location,
        const NavigationStatus& nav_status, size_t instruction_index) const;
    /**
     * Returns true if the specified time in seconds is within the specified
     * lower and upper bounds; otherwise, returns false.
     *
     * @param  time  The time in seconds to verify.
     * @param  lower_bound  The lower time bound.
     * @param  upper_bound  The upper time bound.
     *
     * @return true if the specified time in seconds is within the specified
     * lower and upper bounds; otherwise, returns false
     */
    bool IsTimeWithinBounds(uint32_t time, uint32_t lower_bound,
        uint32_t upper_bound) const;

    /**
     * Returns true if the specified length based on units is within the
     * specified lower and upper bounds; otherwise, returns false.
     *
     * @param  length  The length in units to verify.
     * @param  lower_bound  The lower length bound.
     * @param  upper_bound  The upper length bound.
     *
     * @return true if the specified length based on units is within the
     * specified lower and upper bounds; otherwise, returns false.
     */
    bool IsLengthWithinBounds(float length, float lower_bound,
        float upper_bound) const;

    /**
     * Returns true if a post transition should be announced;
     * otherwise, returns false.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     *
     * @return true if a post transition should be announced;
     * otherwise, returns false.
     */
    bool IsPostTransition(const FixLocation& fix_location,
        const NavigationStatus& nav_status) const;

    /**
     * Returns true if an initial transition alert should be announced;
     * otherwise, returns false. Also, if returning true - the alert length
     * will be populated. The determining criteria for the return values are
     * speed, maneuver length, and remaining maneuver length.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     * @param  alert_length  The returned alert length.
     *
     * @return true if an initial transition alert should be announced;
     * otherwise, returns false. Also, if returning true - the alert length
     * will be populated.
     */
    bool IsInitialTransitionAlert(const FixLocation& fix_location,
        const NavigationStatus& nav_status, float& alert_length) const;

    /**
     * Returns the initial long transition alert length based on the distance units.
     *
     * @return the initial long transition alert length based on the distance units.
     */
    float GetInitialLongTransitionAlertLength() const;

    /**
     * Returns the initial long transition alert lower length based on the distance units.
     *
     * @return the initial long transition alert lower length based on the distance units.
     */
    float GetInitialLongTransitionAlertLowerLength() const;

    /**
     * Returns the initial long transition alert upper length based on the distance units.
     *
     * @return the initial long transition alert upper length based on the distance units.
     */
    float GetInitialLongTransitionAlertUpperLength() const;

    /**
     * Returns the initial long transition alert minimum maneuver length
     * based on the distance units.
     *
     * @return the initial long transition alert minimum maneuver length
     * based on the distance units.
     */
    float GetInitialLongTransitionAlertMinManeuverLength() const;

    /**
     * Returns the initial short transition alert length based on the distance units.
     *
     * @return the initial short transition alert length based on the distance units.
     */
    float GetInitialShortTransitionAlertLength() const;

    /**
     * Returns the initial short transition alert lower length based on the distance units.
     *
     * @return the initial short transition alert lower length based on the distance units.
     */
    float GetInitialShortTransitionAlertLowerLength() const;

    /**
     * Returns the initial short transition alert upper length based on the distance units.
     *
     * @return the initial short transition alert upper length based on the distance units.
     */
    float GetInitialShortTransitionAlertUpperLength() const;

    /**
     * Returns the initial short transition alert minimum maneuver length
     * based on the distance units.
     *
     * @return the initial short transition alert minimum maneuver length
     * based on the distance units.
     */
    float GetInitialShortTransitionAlertMinManeuverLength() const;

    /**
     * Returns true if a final transition alert should be announced;
     * otherwise, returns false. Also, if returning true - the alert length
     * will be populated. The determining criteria for the return values are
     * speed, maneuver length, and remaining maneuver length.
     *
     * @param  fix_location  The current fix location of user.
     * @param  nav_status  The current navigation status.
     * @param  alert_length  The returned alert length.
     *
     * @return true if a final transition alert should be announced;
     * otherwise, returns false. Also, if returning true - the alert length
     * will be populated.
     */
    bool IsFinalTransitionAlert(const FixLocation& fix_location,
        const NavigationStatus& nav_status, float& alert_length) const;

    /**
     * Returns the final long transition alert length based on the distance units.
     *
     * @return the final long transition alert length based on the distance units.
     */
    float GetFinalLongTransitionAlertLength() const;

    /**
     * Returns the final long transition alert lower length based on the distance units.
     *
     * @return the final long transition alert lower length based on the distance units.
     */
    float GetFinalLongTransitionAlertLowerLength() const;

    /**
     * Returns the final long transition alert upper length based on the distance units.
     *
     * @return the final long transition alert upper length based on the distance units.
     */
    float GetFinalLongTransitionAlertUpperLength() const;

    /**
     * Returns the final long transition alert minimum maneuver length
     * based on the distance units.
     *
     * @return the final long transition alert minimum maneuver length
     * based on the distance units.
     */
    float GetFinalLongTransitionAlertMinManeuverLength() const;

    /**
     * Returns the final medium transition alert length based on the distance units.
     *
     * @return the final medium transition alert length based on the distance units.
     */
    float GetFinalMediumTransitionAlertLength() const;

    /**
     * Returns the final medium transition alert lower length based on the distance units.
     *
     * @return the final medium transition alert lower length based on the distance units.
     */
    float GetFinalMediumTransitionAlertLowerLength() const;

    /**
     * Returns the final medium transition alert upper length based on the distance units.
     *
     * @return the final medium transition alert upper length based on the distance units.
     */
    float GetFinalMediumTransitionAlertUpperLength() const;

    /**
     * Returns the final medium transition alert minimum maneuver length
     * based on the distance units.
     *
     * @return the final medium transition alert minimum maneuver length
     * based on the distance units.
     */
    float GetFinalMediumTransitionAlertMinManeuverLength() const;

    /**
     * Returns the final short transition alert length based on the distance units.
     *
     * @return the final short transition alert length based on the distance units.
     */
    float GetFinalShortTransitionAlertLength() const;

    /**
     * Returns the final short transition alert lower length based on the distance units.
     *
     * @return the final short transition alert lower length based on the distance units.
     */
    float GetFinalShortTransitionAlertLowerLength() const;

    /**
     * Returns the final short transition alert upper length based on the distance units.
     *
     * @return the final short transition alert upper length based on the distance units.
     */
    float GetFinalShortTransitionAlertUpperLength() const;

    /////////////////////////////////////////////////////////////////////////////

    // Specified route to navigate
    Route route_;

    // Current route state of navigator
    NavigationStatus_RouteState route_state_;

    // Current leg index
    size_t leg_index_;

    // Current maneuver index
    size_t maneuver_index_;

    // Boolean kilometer unit flag
    bool kilometer_units_;

    // Current leg shape
    std::vector<midgard::PointLL> shape_;

    // Current shape index
    size_t current_shape_index_;

    // Maneuver speeds in units per second
    std::vector<float> maneuver_speeds_;

    // Remaining leg length and time indexed by shape
    std::vector<std::pair<float, uint32_t>> remaining_leg_values_;

    // List of tuples by maneuver index that keeps track of the used instructions
    //     kInitialTransitionAlert
    //     kFinalTransitionAlert
    //     kPreTransition
    //     kPostTransition
    std::vector<std::tuple<bool, bool, bool, bool>> used_instructions_;

};


}
}

#endif  // VALHALLA_TYR_NAVIGATOR_H_
