#ifndef MMP_TRAFFIC_SEGMENT_MATCHER_H_
#define MMP_TRAFFIC_SEGMENT_MATCHER_H_

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <list>
#include <string>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/json.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/map_matcher_factory.h>

namespace valhalla {
namespace meili {

struct interpolation_t {
  baldr::GraphId edge;   // edge id
  float total_distance;  // distance along the path
  float edge_distance;   // ratio of the distance along the edge
  size_t original_index; // index into the original measurements
  double epoch_time;     // seconds from epoch
};

// Matched traffic segment.
struct traffic_segment_t {
  baldr::GraphId segment_id;     // Traffic segment unique Id
  double start_time;             // Begin time along this segment, if < 0 then no begin match
  size_t begin_shape_index;      // Begins at this index of original input
  double end_time;               // End time along this segment, if < 0 then no end match
  size_t end_shape_index;        // Ends at this index of original input
  int length;                    // Length in meters along this segment, if < 0 then no match
  int queue_length;              // Length of any queue from the end of the segment
  bool internal;                 // Is the set of edges making up this segment internal edge types
  std::vector<uint64_t> way_ids; // A list of way ids from the directed edge
};

/**
 * Traffic segment matcher. Allows matching GPS traces to Valhalla edges and
 * then forms the traffic segments associated to those edges.
 */
class TrafficSegmentMatcher {
public:
  /**
   * Constructor.
   * @param  config  Boost property tree - config information.
   */
  TrafficSegmentMatcher(const boost::property_tree::ptree& config);

  /**
   * Do-nothing-destructor
   */
  virtual ~TrafficSegmentMatcher(){};

  /**
   * Matches the GPS trace to Valhalla edges and then associates those
   * to traffic segments.
   * @param   json   GPS trace as JSON.
   * @return  Returns the traffic segments (and times) that the GPS trace
   *          is matched to.
   */
  virtual std::string match(const std::string& json);

  /**
   * Parses the input to the traffic matcher, mainly the trace array
   * @param  request request with data {"trace":[{"lat":0,"lon":0,time:0},...]}
   * @return the list of measurements from the json trace
   */
  static std::vector<Measurement>
  parse_measurements(const Options& options, float default_accuracy, float default_search_radius);

  /**
   * Jsonifies a vector of traffic segments
   * @param  traffic_segments traffic segments to be jsonified
   * @return the jsonified traffic segments in string form
   */
  static std::string serialize(const std::vector<traffic_segment_t>& traffic_segments);

protected:
  /**
   * Updates the matching results include the begin and end points of the edges on the path
   * in doing so it interpolates the times at those points and gives back a distance along
   * the path for each matching result. Essentially you end up with a chronology which includes
   * the nodes of the edges on the path which is the only way to interpolate segment times
   * @param  matches the matched results of the input measurements
   * @param  matcher the matcher used to generate the match
   * @return the interpolation points one for each match result and node of each path edge
   */
  virtual std::list<std::vector<interpolation_t>>
  interpolate_matches(const std::vector<MatchResult>& matches,
                      std::vector<EdgeSegment>& edges,
                      const std::shared_ptr<MapMatcher>& matcher) const;

  /**
   * Compute queue length. Determine where (and if) speed drops below the
   * threshold along a segment.
   * @param  left       Iterator to the start of the interpolated matches.
   * @param  right      Iterator to the end of the interpolated matches.
   * @param  threshold  Speed (m/s) threshold.
   * @return Returns the distance (meters) from the end of the segment where
   *          speed drops below the threshold.
   */
  int compute_queue_length(std::vector<interpolation_t>::const_iterator left,
                           std::vector<interpolation_t>::const_iterator right,
                           const float threshold) const;

  /**
   * Turns updated matching results with their distances into a list of segments with interpolated
   * times
   * @param interpolations the interpolation points
   * @param reader the graph reader with which we can get access to the segments for a given edge
   * @return the vector of traffic segments along the matched path
   */
  virtual std::vector<traffic_segment_t>
  form_segments(const std::list<std::vector<interpolation_t>>& interpolations,
                baldr::GraphReader& reader) const;

  valhalla::meili::MapMatcherFactory matcher_factory;
  std::unordered_set<std::string> customizable;
};

} // namespace meili
} // namespace valhalla
#endif // MMP_TRAFFIC_SEGMENT_MATCHER_H_
