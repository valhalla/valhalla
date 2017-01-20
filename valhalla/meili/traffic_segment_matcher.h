#ifndef MMP_TRAFFIC_SEGMENT_MATCHER_H_
#define MMP_TRAFFIC_SEGMENT_MATCHER_H_

#include <string>
#include <sstream>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/meili/match_route.h>

namespace valhalla {
namespace meili {

// Structure to identify each edge matched to a GPS trace
struct EdgeOnTrace {
  valhalla::baldr::GraphId edge_id;
  float dist;
  float secs;

  EdgeOnTrace(const valhalla::baldr::GraphId& id, const float d,
              const float s)
      : edge_id(id),
        dist(d),
        secs(s) {
  }
};

// Unique edges along the GPS trace
struct UniqueEdgeOnTrace {
  valhalla::baldr::GraphId edge_id;
  float start_pct;
  float end_pct;
  float secs1;
  float secs2;

  UniqueEdgeOnTrace(const valhalla::baldr::GraphId& id, const float p1,
              const float p2, const float t1, const float t2)
      : edge_id(id),
        start_pct(p1),
        end_pct(p2),
        secs1(t1),
        secs2(t2) {
  }
};


// Matched traffic segment.
struct MatchedTrafficSegments {
  bool partial_start;                   // Begins along the segment
  bool partial_end;                     // Ends along the segment
  valhalla::baldr::GraphId segment_id;  // Traffic segment unique Id.
  float begin_time;                     // Begin time along this segment.
  float end_time;                       // End time along this segment.
  uint32_t length;                      // Length in meters along this segment

  MatchedTrafficSegments(const bool start, const bool end,
                         const valhalla::baldr::GraphId& id,
                         const float bt, const float et, const float l)
      : partial_start(start),
        partial_end(end),
        segment_id(id),
        begin_time(bt),
        end_time(et),
        length(l) {
  }

  boost::property_tree::ptree ToPtree() const {
    boost::property_tree::ptree segment;
    segment.put<bool>("partial_start", partial_start);
    segment.put<bool>("partial_end", partial_end);
    segment.put("segment_id", segment_id.value);
    segment.put<float>("begin_time", begin_time);
    segment.put<float>("end_time", end_time);
    segment.put("length", length);
    return segment;
  }
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
   * Matches the GPS trace to Valhalla edges and then associates those
   * to traffic segments.
   * @param   json   GPS trace as JSON.
   * @return  Returns the traffic segments (and times) that the GPS trace
   *          is matched to.
   */
  std::string match(const std::string& json);

 protected:
  valhalla::baldr::GraphReader reader;
  valhalla::meili::MapMatcherFactory matcher_factory;
};

}
}
#endif // MMP_TRAFFIC_SEGMENT_MATCHER_H_
