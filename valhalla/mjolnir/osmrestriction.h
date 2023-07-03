#ifndef VALHALLA_MJOLNIR_OSMRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMRESTRICTION_H

#include <cstdint>
#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>

#include <vector>

namespace valhalla {
namespace mjolnir {

/**
 * OSM restriction information. Result of parsing OSM simple restrictions
 * found in the relations. Restrictions are stored in a multimap keyed by
 * the Id of the "from" way of the restriction.
 */
struct OSMRestriction {
  /**
   * Set the restriction type
   */
  void set_type(baldr::RestrictionType type);

  /**
   * Get the restriction type
   */
  baldr::RestrictionType type() const;

  /**
   * Set the via OSM node id
   */
  void set_via(uint64_t via);

  /**
   * Get the via OSM node id.
   */
  uint64_t via() const;

  /**
   * Set the vias -- used for complex the restrictions.
   */
  void set_vias(const std::vector<uint64_t>& vias);

  /**
   * Get the vias -- used for complex the restrictions.
   */
  std::vector<uint64_t> vias() const;

  /**
   * Set the via node GraphId.
   */
  void set_via(const baldr::GraphId& id);

  /**
   * Get the via GraphId
   */
  const baldr::GraphId& via_graphid() const;

  /**
   * Set the modes
   */
  void set_modes(uint32_t modes);

  /**
   * Get the modes
   */
  uint32_t modes() const;

  /**
   * Set the from way id
   */
  void set_from(uint64_t from);

  /**
   * Get the from way id
   */
  uint64_t from() const;

  /**
   * Set the to way id
   */
  void set_to(uint64_t to);

  /**
   * Get the to way id
   */
  uint64_t to() const;

  /**
   * Set the time domain
   */
  void set_time_domain(uint64_t via);

  /**
   * Get the time domain.
   */
  uint64_t time_domain() const;

  /**
   * Set the probability
   */
  void set_probability(uint8_t probability);

  /**
   * Get the probability.
   */
  uint8_t probability() const;

  /**
   * overloaded < operator - used to sort
   */
  bool operator<(const OSMRestriction& o) const {
    if (from() == o.from()) {
      if (to() == o.to()) {
        if (std::memcmp(vias_, o.vias_, sizeof(vias_)) == 0) {
          if (modes() == o.modes()) {
            if (probability() == o.probability()) {
              return (time_domain() < o.time_domain());
            } else {
              return (probability() < o.probability());
            }
          } else {
            return modes() < o.modes();
          }
        } else {
          return vias() < o.vias();
        }
      } else {
        return to() < o.to();
      }
    } else {
      return from() < o.from();
    }
  }

  /**
   * overloaded == operator - used to compare complex restrictions
   */
  bool operator==(const OSMRestriction& o) const {
    return (from() == o.from() && to() == o.to() && std::memcmp(vias_, o.vias_, sizeof(vias_)) == 0 &&
            modes() == o.modes() && probability() == o.probability() &&
            time_domain() == o.time_domain());
  }

  // from is a way - uses OSM way Id.
  uint64_t from_;

  // to is a way - uses OSM way Id.
  uint64_t to_;

  // Via is a node. When parsing OSM this is stored as an OSM node Id.
  // It later gets changed into a GraphId.
  union ViaNode {
    ViaNode() {
    }
    baldr::GraphId id;
    uint64_t osmid;
  };
  ViaNode via_;

  // fixed size of vias.
  uint64_t vias_[valhalla::baldr::kMaxViasPerRestriction];

  // timed restriction information
  uint64_t time_domain_;

  // Type and time information of the restriction.
  struct Attributes {
    uint32_t type_ : 4;
    uint32_t modes_ : 12;
    uint32_t probability_ : 7;
    uint32_t spare_ : 9;
  };
  Attributes attributes_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMRESTRICTION_H
