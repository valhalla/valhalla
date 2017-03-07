#ifndef MMP_TRAFFIC_SEGMENT_MATCHER_H_
#define MMP_TRAFFIC_SEGMENT_MATCHER_H_

#include <string>
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
struct EdgeOnTrace {
  valhalla::baldr::GraphId edge_id;
  float dist;
  float secs;
};

// Unique edges along the GPS trace
struct UniqueEdgeOnTrace {
  valhalla::baldr::GraphId edge_id;
  float start_pct;
  float end_pct;
  float start_secs;
  float end_secs;
  long int begin_edge_index;
  long int end_edge_index;

};


// Matched traffic segment.
struct MatchedTrafficSegment {
  bool partial_start;                   // Begins along the segment
  bool partial_end;                     // Ends along the segment
  valhalla::baldr::GraphId segment_id;  // Traffic segment unique Id.
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
  static std::vector<meili::Measurement> parse_measurements(const std::string& json,
    float default_accuracy, float default_search_radius);

  /**
   * Jsonifies a vector of traffic segments
   * @param  traffic segments to be jsonified
   * @return the jsonified traffic segments in string form
   */
  static std::string serialize(const std::vector<MatchedTrafficSegment>& traffic_segments);

 protected:

  /**
   * Turns measurements and the matching results into a set of traffic segments
   * @param  the original measurements in the input
   * @param  the matched results given the input measurements
   * @return the vector of traffic segments along the path described by the match results
   */
  virtual std::vector<MatchedTrafficSegment> form_segments(const std::vector<meili::Measurement>& measurements,
    const std::vector<valhalla::meili::MatchResult>& match_results);

  valhalla::meili::MapMatcherFactory matcher_factory;
  valhalla::baldr::GraphReader& reader;
};

}
}
#endif // MMP_TRAFFIC_SEGMENT_MATCHER_H_
