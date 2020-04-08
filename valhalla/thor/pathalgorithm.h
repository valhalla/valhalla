#pragma once

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr size_t kInterruptIterationsInterval = 5000;

/**
 * Pure virtual class defining the interface for PathAlgorithm - the algorithm
 * to create shortest path.
 */
class PathAlgorithm {
public:
  /**
   * Constructor
   */
  PathAlgorithm() : interrupt(nullptr), has_ferry_(false), expansion_callback_() {
  }

  /**
   * Destructor
   */
  virtual ~PathAlgorithm() {
  }

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) = 0;

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() = 0;

  /**
   * Set a callback that will throw when the path computation should be aborted
   * @param interrupt_callback  the function to periodically call to see if
   *                            we should abort
   */
  void set_interrupt(const std::function<void()>* interrupt_callback) {
    interrupt = interrupt_callback;
  }

  /**
   * Does the path include a ferry?
   * @return  Returns true if the path includes a ferry.
   */
  bool has_ferry() const {
    return has_ferry_;
  }

  /**
   * Sets the functor which will track the algorithms expansion.
   *
   * @param  expansion_callback  the functor to call back when the algorithm makes progress
   *                             on a given edge
   */
  using expansion_callback_t =
      std::function<void(baldr::GraphReader&, const char*, baldr::GraphId, const char*, bool)>;
  void set_track_expansion(const expansion_callback_t& expansion_callback) {
    expansion_callback_ = expansion_callback;
  }

protected:
  const std::function<void()>* interrupt;

  bool has_ferry_; // Indicates whether the path has a ferry

  // for tracking the expansion of the algorithm visually
  expansion_callback_t expansion_callback_;

  /**
   * Check for path completion along the same edge. Edge ID in question
   * is along both an origin and destination and origin shows up at the
   * beginning of the edge while the destination shows up at the end of
   * the edge.
   * @param  edgeid       Edge id.
   * @param  origin       Origin path location information.
   * @param  destination  Destination path location information.
   */
  virtual bool IsTrivial(const baldr::GraphId& edgeid,
                         const valhalla::Location& origin,
                         const valhalla::Location& destination) const {
    for (const auto& destination_edge : destination.path_edges()) {
      if (destination_edge.graph_id() == edgeid) {
        for (const auto& origin_edge : origin.path_edges()) {
          if (origin_edge.graph_id() == edgeid &&
              origin_edge.percent_along() <= destination_edge.percent_along()) {
            return true;
          }
        }
      }
    }
    return false;
  }
};

// Container for the data we iterate over in Expand* function
struct EdgeMetadata {
  const baldr::DirectedEdge* edge;
  baldr::GraphId edge_id;
  EdgeStatusInfo* edge_status;

  inline static EdgeMetadata make(const baldr::GraphId& node,
                                  const baldr::NodeInfo* nodeinfo,
                                  const baldr::GraphTile* tile,
                                  EdgeStatus& edge_status_) {
    baldr::GraphId edge_id = {node.tileid(), node.level(), nodeinfo->edge_index()};
    EdgeStatusInfo* edge_status = edge_status_.GetPtr(edge_id, tile);
    const baldr::DirectedEdge* directededge = tile->directededge(edge_id);
    return {directededge, edge_id, edge_status};
  }

  inline void increment_pointers() {
    ++edge;
    ++edge_id;
    ++edge_status;
  }
};

// A structure for tracking time information as the route progresses
struct TimeInfo {
  // whether or not the provided location had valid time information or not
  uint64_t valid : 1;

  // index into the timezone database of the location
  // used to do timezone offset as the route progresses
  uint64_t timezone_index : 9;

  // seconds from epoch adjusted for timezone at the location
  // used to do local time offset as the route progresses
  uint64_t local_time : 54;

  // the ordinal second from the beginning of the week (starting monday at 00:00)
  // used to look up historical traffic as the route progresses
  uint64_t second_of_week : 20;

  // the distance in seconds from now
  uint64_t seconds_from_now : 44;

  /**
   * Helper function to initialize the object from a location. Uses the graph to
   * find timezone information about the edge candidates at the location. If the
   * graph has no timezone information or the location has no edge candidates the
   * default timezone will be used (if unspecified UTC is used). If no datetime
   * is provided on the location or an invalid one is provided the TimeInfo will
   * be invalid.
   *
   * @param location                 location for which to initialize the TimeInfo
   * @param reader                   used to get timezone information from the graph
   * @param default_timezone_index   used when no timezone information is available
   * @return the initialized TimeInfo
   */
  static TimeInfo make(valhalla::Location& location,
                       baldr::GraphReader& reader,
                       int default_timezone_index = baldr::DateTime::get_tz_db().to_index("Etc/UTC"));

  /**
   * Offset all the initial time info to reflect the progress along the route to this point
   * @param seconds_offset  the number of seconds to offset the TimeInfo by
   * @param next_tz_index   the timezone index at the new location
   * @return a new TimeInfo object reflecting the offset
   */
  inline TimeInfo forward(float seconds_offset, int next_tz_index) const {
    namespace dt = baldr::DateTime;

    if (!valid)
      return *this;

    // offset the local time and second of week by the amount traveled to this label
    uint64_t lt = local_time + static_cast<uint64_t>(seconds_offset);
    uint32_t sw = static_cast<uint32_t>(second_of_week + seconds_offset);

    // if the timezone changed we need to account for that offset as well
    if (next_tz_index != timezone_index) {
      int tz_diff = dt::timezone_diff(lt, dt::get_tz_db().from_index(timezone_index),
                                      dt::get_tz_db().from_index(next_tz_index));
      lt += tz_diff;
      sw += tz_diff;
    }

    // wrap the week second if it went past the end
    if (sw > valhalla::midgard::kSecondsPerWeek) {
      sw -= valhalla::midgard::kSecondsPerWeek;
    }

    // return the shifted object, notice that seconds from now is only useful for
    // date_time type == current
    return {valid, static_cast<uint64_t>(next_tz_index), lt, sw,
            seconds_from_now + static_cast<int64_t>(seconds_offset)};
  }

  /**
   * Offset all the initial time info to reflect the progress along the route to this point
   * @param seconds_offset  the number of seconds to offset the TimeInfo by
   * @param next_tz_index   the timezone index at the new location
   * @return a new TimeInfo object reflecting the offset
   */
  inline TimeInfo reverse(float seconds_offset, int next_tz_index) const {
    namespace dt = baldr::DateTime;

    if (!valid)
      return *this;

    // offset the local time and second of week by the amount traveled to this label
    uint64_t lt = local_time - static_cast<uint64_t>(seconds_offset); // dont route near the epoch
    int32_t sw = static_cast<int32_t>(second_of_week) - static_cast<int32_t>(seconds_offset);

    // if the timezone changed we need to account for that offset as well
    if (next_tz_index != timezone_index) {
      int tz_diff = dt::timezone_diff(lt, dt::get_tz_db().from_index(timezone_index),
                                      dt::get_tz_db().from_index(next_tz_index));
      lt += tz_diff;
      sw += tz_diff;
    }

    // wrap the week second if it went past the beginning
    if (sw < 0) {
      sw = valhalla::midgard::kSecondsPerWeek + sw;
    }

    // return the shifted object, notice that seconds from now is negative, this could be useful if
    // we had the ability to arrive_by current time but we dont for the moment
    return {valid, static_cast<uint64_t>(next_tz_index), lt, static_cast<uint32_t>(sw),
            seconds_from_now - static_cast<int64_t>(seconds_offset)};
  }

  // for unit tests
  bool operator==(const TimeInfo& ti) const {
    return valid == ti.valid && timezone_index == ti.timezone_index && local_time == ti.local_time &&
           second_of_week == ti.second_of_week && seconds_from_now == ti.seconds_from_now;
  }

  // for unit tests
  friend std::ostream& operator<<(std::ostream& os, const TimeInfo& ti) {
    return os << "{valid: " << ti.valid << ", timezone_index: " << ti.timezone_index
              << ", local_time: " << ti.local_time << ", second_of_week: " << ti.second_of_week
              << ", seconds_from_now: " << ti.seconds_from_now << "}";
  }
};

} // namespace thor
} // namespace valhalla
