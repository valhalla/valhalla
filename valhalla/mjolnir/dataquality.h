#ifndef VALHALLA_MJOLNIR_DATAQUALITY_H
#define VALHALLA_MJOLNIR_DATAQUALITY_H

#include <algorithm>
#include <cstdint>
#include <map>
#include <unordered_set>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace mjolnir {

enum DataIssueType {
  kDuplicateWays = 0,       // 2 ways that overlap between same 2 nodes
  kUnconnectedLinkEdge = 1, // Link (ramp) that is unconnected
  kIncompatibleLinkUse = 2  // Link (ramp) that has incompatible use (e.g. driveway)
};

// Simple struct for holding duplicate ways to allow sorting by edgecount
struct DuplicateWay {
  uint32_t wayid1;
  uint32_t wayid2;
  uint32_t edgecount;

  DuplicateWay(const uint32_t id1, const uint32_t id2, const uint32_t n)
      : wayid1(id1), wayid2(id2), edgecount(n) {
  }

  // For sorting by number of duplicate edges
  bool operator<(const DuplicateWay& other) const {
    return edgecount > other.edgecount;
  }
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
   * Add statistics (accumulate from several DataQuality objects)
   * @param  stats  Data quality object to add to stats
   */
  void AddStatistics(const DataQuality& stats);

  /**
   * Adds an issue.
   */
  void AddIssue(const DataIssueType issuetype,
                const baldr::GraphId& graphid,
                const uint32_t wayid1,
                const uint32_t wayid2);

  /**
   * Log simple statistics.
   */
  void LogStatistics() const;

  /**
   * Log issues.
   */
  void LogIssues() const;

  // Public - simple stats
  uint32_t nodecount;
  uint32_t directededge_count;
  uint32_t edgeinfocount;
  uint32_t simplerestrictions;
  uint32_t timedrestrictions;
  uint32_t culdesaccount;
  uint32_t forward_restrictions_count;
  uint32_t reverse_restrictions_count;
  uint32_t node_counts[128];

protected:
  // Unconnected links
  std::unordered_set<uint32_t> unconnectedlinks_;

  // Unconnected links
  std::unordered_set<uint32_t> incompatiblelinkuse_;

  // Duplicate way Ids
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> duplicateways_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_DATAQUALITY_H
