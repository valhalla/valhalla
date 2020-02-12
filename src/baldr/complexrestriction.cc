#include "baldr/complexrestriction.h"

namespace valhalla {
namespace baldr {

bool CheckPatchPathForRestrictions(
    const std::vector<valhalla::baldr::GraphId>& patch_path,
    const std::vector<std::vector<valhalla::baldr::GraphId>>& list_of_restrictions) {
  // const auto full_patch_begin = patch_path.cbegin();
  for (auto& restriction_ids : list_of_restrictions) {
    auto restriction_id = restriction_ids.cbegin();
    for (auto org_edge_id = patch_path.cbegin(); org_edge_id != patch_path.cend(); ++org_edge_id) {
      if (*restriction_id == *org_edge_id) {
        // Found beginning restriction in patch_path, check if the remaining vias match
        auto edge_id = org_edge_id;
        bool found_match = true;
        while (true) {
          ++restriction_id;
          ++edge_id;
          if (restriction_id == restriction_ids.cend()) {
            break;
          }
          if (*restriction_id != *edge_id || edge_id == patch_path.cend()) {
            found_match = false;
            break;
          }
        }
        if (found_match) {
          // We found a matching restriction, return early
          return true;
        }
      }
    }
  }
  // None of the restrictions matched
  return false;
};

} // namespace baldr
} // namespace valhalla
