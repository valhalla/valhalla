#ifndef VALHALLA_THOR_EXPANDFROMNODE_H_
#define VALHALLA_THOR_EXPANDFROMNODE_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>



namespace valhalla {
namespace thor {

class ExpandFromNode {
 public:

  static bool FormPath(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                       const sif::TravelMode& mode, baldr::GraphReader& reader,
                       const std::vector<midgard::PointLL>& shape,
                       size_t& correlated_index, const baldr::GraphTile* tile,
                       const baldr::GraphId& node,
                       const baldr::GraphId& stop_node,
                       sif::EdgeLabel& prev_edge_label, float& elapsed_time,
                       std::vector<PathInfo>& path_infos,
                       const bool from_transition);
};

}
}

#endif  // VALHALLA_THOR_EXPANDFROMNODE_H_
