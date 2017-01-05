#include <string>
#include <sstream>
#include <boost/python.hpp>
#include <boost/python/str.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/extract.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/meili/match_route.h>

namespace {

  //statically set the config file and configure logging, throw if you never configured
  //configuring multiple times is wasteful/ineffectual but not harmful
  boost::property_tree::ptree configure(const boost::optional<std::string>& config = boost::none) {
    static boost::optional<boost::property_tree::ptree> pt;
    LOG_INFO("INSIDE OF CONFIGURE");
    //if we haven't already loaded one
    if(config && !pt) {
      try {
        //parse the config
        boost::property_tree::ptree temp_pt;
        boost::property_tree::read_json(config.get(), temp_pt);
        pt = temp_pt;

        //configure logging
        boost::optional<boost::property_tree::ptree&> logging_subtree = pt->get_child_optional("tyr.logging");
        if(logging_subtree) {
          auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
          valhalla::midgard::logging::Configure(logging_config);
        }
      }
      catch(...) {
        throw std::runtime_error("Failed to load config from: " + config.get());
      }
    }

    //if it turned out no one ever configured us we throw
    if(!pt)
      throw std::runtime_error("The service was not configured");
    return *pt;
  }
  void py_configure(const std::string& config_file) {
    configure(config_file);
  }

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

  //wrapper class around valhalla objects we'll use to do segment matching
  class segment_matcher {
   public:
    segment_matcher()
       : reader(configure().get_child("mjolnir")),
         matcher_factory(configure()) {
    }

    // Use map-matching to form list of traffic segments on this tace
    std::string match(const std::string& json) {
      // From ptree from JSON string
      boost::property_tree::ptree request;
      try {
        std::stringstream stream(json);
        boost::property_tree::read_json(stream, request);
      } catch (...) {
        LOG_ERROR("Error parsing JSON= " + json);
        return "{\"foo\":\"bar\"}";
      }

      // Form trace positions
      std::vector<PointLL> trace;
      std::vector<uint32_t> times;
      auto trace_pt = request.get_child_optional("trace");

      if (trace_pt) {
        for (const auto& lls : *trace_pt) {
          float lat = lls.second.get<float>("lat");
          float lon = lls.second.get<float>("lon");
          trace.emplace_back(lon, lat);
          times.push_back(lls.second.get<int>("time"));
        }
      } else {
        LOG_ERROR("Could not form trace from input JSON= " + json);
        return "{\"foo\":\"bar\"}";
      }

      LOG_INFO("trace size = " + std::to_string(trace.size()));

      // Call Meili for map matching to get a collection of pathLocation Edges
      // Create a matcher
      std::shared_ptr<valhalla::meili::MapMatcher> matcher;
      try {
        matcher.reset(matcher_factory.Create(trace_config));
      } catch (const std::invalid_argument& ex) {
        // TODO - what to return?
        return "{\"foo\":\"bar\"}";
      }

      // Populate a measurement sequence to pass to the map matcher
      std::vector<valhalla::meili::Measurement> sequence;
      for (const auto& coord : trace) {
        sequence.emplace_back(coord,
                              matcher->config().get<float>("gps_accuracy"),
                              matcher->config().get<float>("search_radius"));
      }

      // Create the vector of matched path results
      std::vector<valhalla::meili::MatchResult> results;
      if (sequence.size() > 0) {
        results = matcher->OfflineMatch(sequence);
      }

      if (sequence.size() != results.size()) {
        LOG_ERROR("Sequence size not equal to match reslt size");
        return "{\"foo\":\"bar\"}";
      }

      // TODO - more robust list of edges. Handle cases where multiple
      // edges lie on the path between GPS locations (sequence) and
      // handle cases where discontinuities arise

      // Form a list of edges for each result - match times to each.
      size_t idx = 0;
      std::vector<EdgeOnTrace> trace_edges;
      for (auto res : results) {
        // Make sure edge is valid
        if (res.edgeid().Is_Valid()) {
          trace_edges.emplace_back(res.edgeid(), GetEdgeDist(res, matcher), times[idx]);
        }
        idx++;
      }

      // Iterate through the edges and form a list of unique edges
      valhalla::baldr::GraphId prior_edge;
      std::vector<UniqueEdgeOnTrace> edges;
      for (auto edge : trace_edges) {
        if (!prior_edge.Is_Valid()) {
          edges.emplace_back(edge.edge_id, edge.dist, edge.dist, edge.secs, edge.secs);
        } else  if (edge.edge_id == prior_edge) {
          // Update the time at the end
          edges.back().end_pct = edge.dist;
          edges.back().secs2   = edge.secs;
        } else {
          // TODO - verify that the edges are connected.
          // what if these edges are not connected?

          // Update time and distance at the end of the prior edge
          edges.back().end_pct = 1.0f;
          edges.back().secs2   = edge.secs ;

          // New edge
          edges.emplace_back(edge.edge_id, 0.0f, edge.dist, edge.secs, edge.secs);
        }
        prior_edge = edge.edge_id;
      }

/*      for (const auto& edge : edges) {
        LOG_INFO("Edge: begin pct = " + std::to_string(edge.start_pct) + " end pct = "
                        + std::to_string(edge.end_pct) +
                        " t1 = " + std::to_string(edge.secs1) +
                        " t2 = " + std::to_string(edge.secs2));
      } */

      valhalla::baldr::GraphId prior_segment;
      std::vector<MatchedTrafficSegments> traffic_segment;
      for (const auto& edge : edges) {
        // Get the directed edge Id and tile
        valhalla::baldr::GraphId edge_id = edge.edge_id;
        const valhalla::baldr::GraphTile* tile = matcher->graphreader().GetGraphTile(edge_id);
        const valhalla::baldr::DirectedEdge* directededge = tile->directededge(edge_id);
        float length = directededge->length() * (edge.end_pct - edge.start_pct);

        // Get the traffic segment(s) associated to this edge
        float begin_time, end_time;
        auto segments = tile->GetTrafficSegments(edge_id);
        if (segments.size() > 0) {
          for (const auto& seg : segments) {
            bool starts = seg.first.starts_segment();
            bool ends   = seg.first.ends_segment();
            float p1 = seg.first.begin_percent();
            float p2 = seg.first.end_percent();
            valhalla::baldr::GraphId segment_id = seg.first.segment_id();
            float weight = seg.second;

/*            LOG_INFO("segment_id " + std::to_string(segment_id.value) +
                     " starts = " + std::to_string(starts) +
                     " ends = " + std::to_string(ends) +
                     " p1 = " + std::to_string(p1) +
                     " p2 = " + std::to_string(p2)); */

             if (segment_id == prior_segment) {
               // Update end time, end_pct, and length of the current segment
               traffic_segment.back().end_time = edge.secs2;
               traffic_segment.back().length += length;
               if (seg.first.ends_segment() && edge.end_pct == 1.0f) {
                 traffic_segment.back().partial_end = false;
               }
             } else if (segment_id.value == 0) {
               ; // LOG_INFO("Traffic segment ID == 0");
             } else {
               bool starts = (seg.first.starts_segment() && edge.start_pct == 0.0f);
               bool ends =   (seg.first.ends_segment() && edge.end_pct == 1.0f);
               traffic_segment.emplace_back(!starts,
                           !seg.first.ends_segment(), segment_id, edge.secs1,
                            edge.secs2, length);
               prior_segment = segment_id;
             }
           }
         } else {
           // No traffic segment associated to this edge
           if (directededge->classification() <= valhalla::baldr::RoadClass::kTertiary) {
             LOG_ERROR("No traffic associated to this edge");
           }
         }
         prior_edge = edge_id;
      }

      // Serialize and return as a string
      boost::property_tree::ptree result;
      boost::property_tree::ptree segments;
      for (const auto& seg : traffic_segment) {
        segments.push_back(std::make_pair("", seg.ToPtree()));
      }
      result.put_child("segments", segments);
      std::stringstream ss;
      boost::property_tree::write_json(ss, result);
      return ss.str();
    }

   protected:
    valhalla::baldr::GraphReader reader;
    valhalla::meili::MapMatcherFactory matcher_factory;
    boost::property_tree::ptree trace_config;            // TODO - where does this come from?

    float GetEdgeDist(const valhalla::meili::MatchResult& res,
                      std::shared_ptr<valhalla::meili::MapMatcher> matcher) {
      if (res.HasState()) {
        bool found = false;
        const auto& state = matcher->mapmatching().state(res.stateid());
        valhalla::baldr::PathLocation loc = state.candidate();
        for (const auto& e : loc.edges) {
         if (e.id == res.edgeid()) {
           return e.dist;
         }
        }
      }
      return 1.0f;
    }
  };

}

BOOST_PYTHON_MODULE(valhalla) {

  //python interface for configuring the system, always call this first in your python program
  boost::python::def("Configure", py_configure);

  //class for doing matching, a dummy for now but can be fleshed out
  boost::python::class_<segment_matcher,
      boost::shared_ptr<segment_matcher>,
      boost::noncopyable>("SegmentMatcher", boost::python::no_init)
    .def("__init__", boost::python::make_constructor(&boost::make_shared<segment_matcher>))
    .def("Match", &segment_matcher::match)
  ;

}
