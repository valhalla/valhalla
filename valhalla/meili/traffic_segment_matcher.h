#ifndef MMP_TRAFFIC_SEGMENT_MATCHER_H_
#define MMP_TRAFFIC_SEGMENT_MATCHER_H_

#include <string>
#include <vector>
#include <sstream>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphid.h"
#include "baldr/json.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "meili/match_route.h"

namespace valhalla {
namespace meili {

// Structure to identify each edge matched to a GPS trace
struct trace_edge_t {
  baldr::GraphId edge_id;      //the id for the edge
  float begin_dist_offset;     //distance from first trace point along entire path to beginning of edge
  float end_dist_offset;       //distance from first trace point along entire path to end of edge
  float begin_time_offset;     //time from first trace point along entire path to beginning of edge
  float end_time_offset;       //time from first trace point along entire path to end of edge
};


// Matched traffic segment.
struct MatchedTrafficSegment {
  bool partial_start;                   // Begins along the segment
  bool partial_end;                     // Ends along the segment
  baldr::GraphId segment_id;            // Traffic segment unique Id.
  float start_time;                     // Begin time along this segment.
  float end_time;                       // End time along this segment.
  uint32_t length;                      // Length in meters along this segment
  long int begin_shape_index;
  long int end_shape_index;
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
  virtual ~TrafficSegmentMatcher() {};

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
   * @param  json string of gps data {"trace":[{"lat":0,"lon":0,time:0},...]}
   * @return the list of measurements from the json trace
   */
  static std::vector<Measurement> parse_measurements(const std::string& json,
    float default_accuracy, float default_search_radius);

  /**
   * Jsonifies a vector of traffic segments
   * @param  traffic segments to be jsonified
   * @return the jsonified traffic segments in string form
   */
  static std::string serialize(const std::vector<MatchedTrafficSegment>& traffic_segments);

 protected:

  /**
   * Turns the matching results into a set of edges forming the path the input measurements traversed
   * it also injects fake match results corresponding to the end points of the path edges can be used
   * as a chronology against which we can interpolate the traffic segment end points
   * @param  the matched results of the input measurements
   * @param  the matcher used to generate the match
   * @return the edges along the matched path
   */
  virtual std::vector<trace_edge_t> form_edges(std::vector<MatchResult>& match_results,
    const std::shared_ptr<MapMatcher>& matcher) const;

  /**
   * Turns trace edges and matching results into a set of traffic segments
   * @return the vector of traffic segments along the path described by the match results
   */
  virtual std::vector<MatchedTrafficSegment> form_segments(const std::vector<trace_edge_t>& trace_edges,
    const std::vector<MatchResult>& match_results) const;

  valhalla::meili::MapMatcherFactory matcher_factory;
};

}
}
#endif // MMP_TRAFFIC_SEGMENT_MATCHER_H_
