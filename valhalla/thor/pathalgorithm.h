#pragma once

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/pathinfo.h>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

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
  PathAlgorithm() : interrupt(nullptr), has_ferry_(false), track_expansion_(false) {
    expansion_.SetObject();
    rapidjson::Pointer("/type").Set(expansion_, "FeatureCollection");
    rapidjson::Pointer("/features").Create(expansion_).SetArray();
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
   * Returns a geojson feature collection where each feature represents
   * and edge that was visited by the algorithm. The order of the features
   * is the same order in which the edges were visited. Each feature/edge
   * will have properties describing the edge's id as well as the current
   * state (reached, settled, connected) so that one can replay the expansion
   * as a time series and visually inspect what the algorithm is doing.
   *
   * This method will return a feature collection after a call to GetBestPath.
   * If the underlying path algorithm doesn't implement this or GetBestPath
   * has not been called an empty feature collection is returned.
   *
   * @return the feature collection
   */
  const rapidjson::Document& GetPathExpansionHistory() const {
    return expansion_;
  }

protected:
  const std::function<void()>* interrupt;

  bool has_ferry_; // Indicates whether the path has a ferry

  // for tracking the expansion of the algorithm visually
  bool track_expansion_;
  rapidjson::Document expansion_;

  /**
   * Check for path completion along the same edge. Edge ID in question
   * is along both an origin and destination and origin shows up at the
   * beginning of the edge while the destination shows up at the end of
   * the edge.
   * @param  edgeid       Edge id.
   * @param  origin       Origin path location information.
   * @param  destination  Destination path location information.
   */
  bool IsTrivial(const baldr::GraphId& edgeid,
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

  /**
   * Convenience method to get the timezone index at a node.
   * @param graphreader Graph reader.
   * @param node GraphId of the node to get the timezone index.
   * @return Returns the timezone index. A value of 0 indicates an invalid timezone.
   */
  int GetTimezone(baldr::GraphReader& graphreader, const baldr::GraphId& node) {
    const baldr::GraphTile* tile = graphreader.GetGraphTile(node);
    return (tile == nullptr) ? 0 : tile->node(node)->timezone();
  }

  /**
   * Adds an edge to the expansion history of the current pass of the algorithm
   * There is no schema for the status, but loosely you should pass one of:
   * reached, settled, connected so the person using this geojson can style accordingly
   */
  virtual void TrackExpansion(baldr::GraphReader& reader,
                              baldr::GraphId edgeid,
                              const char* status,
                              bool full_shape = false) {
    // full shape might be overkill but meh, its trace
    const auto* tile = reader.GetGraphTile(edgeid);
    const auto* edge = tile->directededge(edgeid);
    auto shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
    if (!edge->forward())
      std::reverse(shape.begin(), shape.end());
    if (!full_shape && shape.size() > 2)
      shape.erase(shape.begin() + 1, shape.end() - 1);

    // make the feature
    auto& a = expansion_.GetAllocator();
    auto* features = rapidjson::Pointer("/features").Get(expansion_);
    features->GetArray().PushBack(rapidjson::Value(rapidjson::kObjectType), a);
    auto& feature = (*features)[features->Size() - 1];
    rapidjson::Pointer("/type").Set(feature, "Feature", a);

    // make the geom
    rapidjson::Pointer("/geometry/type").Set(feature, "LineString", a);
    auto& coords = rapidjson::Pointer("/geometry/coordinates").Create(feature, a).SetArray();
    for (const auto& p : shape) {
      coords.GetArray().PushBack(rapidjson::Value(rapidjson::kArrayType), a);
      auto point = coords[coords.Size() - 1].GetArray();
      point.PushBack(p.first, a);
      point.PushBack(p.second, a);
    }

    // make the properties
    rapidjson::Pointer("/properties/edge").Set(feature, static_cast<uint64_t>(edgeid), a);
    rapidjson::Pointer("/properties/status").Set(feature, status, a);
  }
};

} // namespace thor
} // namespace valhalla
