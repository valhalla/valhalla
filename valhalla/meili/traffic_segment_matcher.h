#ifndef MMP_TRAFFIC_SEGMENT_MATCHER_H_
#define MMP_TRAFFIC_SEGMENT_MATCHER_H_

#include <string>
#include <vector>
#include <list>
#include <sstream>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphfsreader.h"
#include "baldr/graphid.h"
#include "baldr/json.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "meili/match_route.h"

namespace valhalla {
namespace meili {

struct interpolation_t {
  baldr::GraphId edge;   //edge id
  float total_distance;  //distance along the path
  float edge_distance;   //ratio of the distance along the edge
  size_t original_index; //index into the original measurements
  double epoch_time;     //seconds from epoch
};

// Matched traffic segment.
struct traffic_segment_t {
  baldr::GraphId segment_id;   // Traffic segment unique Id
  double start_time;           // Begin time along this segment, if < 0 then no begin match
  size_t begin_shape_index;    // Begins at this index of original input
  double end_time;             // End time along this segment, if < 0 then no end match
  size_t end_shape_index;      // Ends at this index of original input
  int length;                  // Length in meters along this segment, if < 0 then no match
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
  static std::string serialize(const std::vector<traffic_segment_t>& traffic_segments);

 protected:

  /**
   * Updates the matching results include the begin and end points of the edges on the path
   * in doing so it interpolates the times at those points and gives back a distance along
   * the path for each matching result. Essentially you end up with a chronology which includes
   * the nodes of the edges on the path which is the only way to interpolate segment times
   * @param  the matched results of the input measurements
   * @param  the matcher used to generate the match
   * @return the interpolation points one for each match result and node of each path edge
   */
  virtual std::list<std::vector<interpolation_t> > interpolate_matches(const std::vector<MatchResult>& matches,
    const std::shared_ptr<MapMatcher>& matcher) const;

  /**
   * Turns updated matching results with their distances into a list of segments with interpolated times
   * @param  the updated matched results including the nodes of all the edges on the path
   * @param  the distance along the entire path of each matched result
   * @param  the graph reader with which we can get access to the segments for a given edge
   * @return the vector of traffic segments along the matched path
   */
  virtual std::vector<traffic_segment_t> form_segments(const std::list<std::vector<interpolation_t> >& interpolations,
    baldr::GraphReader& reader) const;

  valhalla::meili::MapMatcherFactory matcher_factory;
};

}
}
#endif // MMP_TRAFFIC_SEGMENT_MATCHER_H_
