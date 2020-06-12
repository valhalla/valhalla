#pragma once
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>

#include <functional>
#include <utility>

namespace valhalla {
namespace sif {

// edge id, mode of travel, whether or not this is the last edge
using PathEdge = std::tuple<baldr::GraphId, TravelMode, bool>;
// what this function calls to get the next edge
using EdgeCallback = std::function<PathEdge(void)>;
// what this function calls to emit the next label
using LabelCallback = std::function<void(const EdgeLabel& label)>;

/**
 * Will take a sequence of edges and create the set of edge labels that would represent it
 * Allows for the caller to essentially re-compute the costing of a given path
 *
 * @param reader            used to get access to graph data. modifyable because its got a cache
 * @param origin            the location on the first edge of the path. modifyable because current
 *                          time recosting requires the setting of a date and time
 * @param destination       the location on the last edge of the path
 * @param mode_costing      per travel mode costing objects to be used for costing computations
 * @param edge_cb           the callback used to get each edge in the path
 * @param label_cb          the callback used to emit each label in the path
 */
void recost_forward(baldr::GraphReader& reader,
                    valhalla::Location& origin,
                    const valhalla::Location& destination,
                    const std::shared_ptr<DynamicCost>* mode_costing,
                    const EdgeCallback& edge_cb,
                    const LabelCallback& label_cb) {
  // keep grabbing edges while we get valid ids
  PathEdge edge;
  EdgeLabel label;
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  auto time_info = baldr::TimeInfo::make(origin, reader, &tz_cache);
  while (std::get<0>(edge = edge_cb()).Is_Valid()) {
    // is this the first edge
    auto prev_label = label;
    if (prev_label.predecessor() == baldr::kInvalidLabel) {
      // edge cost * partial distance
    } // this is not the first edge
    else {
      // time update
      // turn cost
      // edge cost * partial distance if its the last edge by id assuming no loops?
    }
    label_cb(label);
  }
}

} // namespace sif
} // namespace valhalla