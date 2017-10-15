#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

// OSM Access.  User set access flags
struct OSMAccess {

  /**
   * Constructor
   */
  OSMAccess();

  /**
   * Constructor with way id arg.
   * @param   id  way id
   */
  OSMAccess(const uint64_t id);

  /**
   * Set way id.
   * @param   id  way id
   */
  void set_way_id(const uint64_t id);

  /**
   * Get the way id
   * @return  Returns way id.
   */
  uint64_t way_id() const;

  /**
   * Sets the auto_tag flag.
   * @param  auto_tag   Autos allowed on this way?
   */
  void set_auto_tag(const bool auto_tag);

  /**
   * Get the auto_tag flag.
   * @return  Returns auto_tag flag.
   */
  bool auto_tag() const;

  /**
   * Sets the bike_tag flag.
   * @param  bike_tag   Bicycles allowed on this way?
   */
  void set_bike_tag(const bool bike_tag);

  /**
   * Get the bike_tag flag.
   * @return  Returns bike_tag flag.
   */
  bool bike_tag() const;

  /**
   * Sets the moped_tag flag.
   * @param  moped_tag  Mopeds/Electric scooters allowed on this way?
   */
  void set_moped_tag(const bool moped_tag);

  /**
   * Get the moped_tag flag.
   * @return  Returns moped_tag flag.
   */
  bool moped_tag() const;

  /**
   * Sets the bus_tag flag.
   * @param  bus_tag    Buses allowed on this way?
   */
  void set_bus_tag(const bool bus_tag);

  /**
   * Get the bus_tag flag.
   * @return  Returns bus_tag flag.
   */
  bool bus_tag() const;

  /**
   * Sets the foot_tag flag.
   * @param  foot_tag   Pedestrians allowed on this way?
   */
  void set_foot_tag(const bool foot_tag);

  /**
   * Get the foot_tag flag.
   * @return  Returns foot_tag flag.
   */
  bool foot_tag() const;

  /**
   * Sets the truck_tag flag.
   * @param  truck_tag    Trucks allowed on this way?
   */
  void set_truck_tag(const bool truck_tag);

  /**
   * Get the truck_tag flag.
   * @return  Returns truck_tag flag.
   */
  bool truck_tag() const;

  /**
   * Sets the hov_tag flag.
   * @param  hov_tag    hov way?
   */
  void set_hov_tag(const bool hov_tag);

  /**
   * Get the hov_tag flag.
   * @return  Returns hov_tag flag.
   */
  bool hov_tag() const;

  /**
   * Sets the motorroad_tag flag.
   * @param  motorroad_tag    motorroad tag exists?
   */
  void set_motorroad_tag(const bool motorroad_tag);

  /**
   * Get the motorroad_tag flag.
   * @return  Returns motorroad_tag flag.
   */
  bool motorroad_tag() const;

  // OSM way Id
  uint64_t osmwayid_;

  // Access attributes
  union AccessAttributes {
    struct Fields {
      uint8_t auto_tag      :1;
      uint8_t bike_tag      :1;
      uint8_t bus_tag       :1;
      uint8_t foot_tag      :1;
      uint8_t truck_tag     :1;
      uint8_t hov_tag       :1;
      uint8_t motorroad_tag :1;
      uint8_t moped_tag     :1;
    } fields;
    uint32_t v;
  };
  AccessAttributes attributes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H
