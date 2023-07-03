#ifndef VALHALLA_MJOLNIR_COUNTRY_ACCESS_H_
#define VALHALLA_MJOLNIR_COUNTRY_ACCESS_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/mjolnir/adminconstants.h>
#include <valhalla/mjolnir/osmaccess.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Removes or adds access.
 * @param  current_access   Current access for the DE.
 * @param  country_access   Country specific access.
 * @param  type             Type of access to add or remove?
 */
uint32_t
ProcessAccess(const uint32_t current_access, const uint32_t country_access, const uint32_t type);

/**
 * Get the new access for a DE.  If a user entered tags then we will not update
 * the access.  Also, we have to take into account if the DE is oneway or not.
 * @param  current_access     Current access for the DE.
 * @param  country_access     Country specific access.
 * @param  oneway_vehicle     Is the DE oneway in the opposite direction for vehicles?
 * @param  oneway_bicycle     Is the DE oneway in the opposite direction for bicycles?
 * @param  oneway_pedestrian  Is the DE oneway in the opposite direction for pedestrians?
 * @param  target             User entered access tags.
 */
uint32_t GetAccess(const uint32_t current_access,
                   const uint32_t country_access,
                   const bool oneway_vehicle,
                   const bool oneway_bicycle,
                   const bool oneway_pedestrian,
                   const OSMAccess& target);

/**
 * Set the country access for a DE.
 * @param  directededge     Current DE.
 * @param  country_access   Country specific access.
 * @param  user_access      User entered access tags.
 */
void SetCountryAccess(DirectedEdge& directededge,
                      const std::vector<int>& country_access,
                      const OSMAccess& user_access);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_COUNTRY_ACCESS_H_
