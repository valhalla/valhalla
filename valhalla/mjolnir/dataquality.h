#ifndef VALHALLA_MJOLNIR_DATAQUALITY_H
#define VALHALLA_MJOLNIR_DATAQUALITY_H

#include <algorithm>
#include <unordered_set>
#include <map>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>

namespace valhalla {
namespace mjolnir {

enum DataIssueType {
  kDuplicateWays       = 0,   // 2 ways that overlap between same 2 nodes
  kUnconnectedLinkEdge = 1,   // Link (ramp) that is unconnected
  kIncompatibleLinkUse = 2    // Link (ramp) that has incompatible use (e.g. driveway)
};

/**
 * Class used to generate statistics and gather data quality issues.
 * Also forms per tile data quality metrics.
 */
class DataQuality {
 public:
  /**
   * Constructor
   */
  DataQuality();

  /**
   * Add statistics for a directed edge.
   */
  void AddStats(const baldr::GraphId& tileid, const baldr::DirectedEdge& edge);

  /**
   * Adds an issue.
   */
  void AddIssue(const DataIssueType issuetype, const baldr::GraphId& graphid,
                const uint64_t wayid1, const uint64_t wayid2);

  /**
   * Log.
   */
  void Log();

protected:
  uint32_t not_thru_count_;

  // Unconnected links
  std::unordered_set<uint64_t> unconnectedlinks_;

  // Unconnected links
  std::unordered_set<uint64_t> incompatiblelinkuse_;

  // Duplicate way Ids
  std::map<std::pair<uint64_t, uint64_t>, uint32_t> duplicateways_;
};

}
}

#endif  // VALHALLA_MJOLNIR_DATAQUALITY_H
