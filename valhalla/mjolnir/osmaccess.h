#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H

#include <cstdint>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

// OSM Access.  User set access flags
struct OSMAccess {

  /**
   * Constructor
   */
  OSMAccess() {
    memset(this, 0, sizeof(OSMAccess));
    osmwayid_ = 0;
    attributes_.v = 0;
  }

  /**
   * Constructor with way id arg.
   * @param   id  way id
   */
  OSMAccess(const uint64_t id) {
    memset(this, 0, sizeof(OSMAccess));
    set_way_id(id);
    attributes_.v = 0;
  }

  /**
   * Set way id.
   * @param   id  way id
   */
  void set_way_id(const uint64_t id) {
    osmwayid_ = id;
  }

  /**
   * Get the way id
   * @return  Returns way id.
   */
  uint64_t way_id() const {
    return osmwayid_;
  }

  /**
   * Sets the auto_tag flag.
   * @param  auto_tag   Autos allowed on this way?
   */
  void set_auto_tag(const bool auto_tag) {
    attributes_.fields.auto_tag = auto_tag;
  }

  /**
   * Get the auto_tag flag.
   * @return  Returns auto_tag flag.
   */
  bool auto_tag() const {
    return attributes_.fields.auto_tag;
  }

  /**
   * Sets the bike_tag flag.
   * @param  bike_tag   Bicycles allowed on this way?
   */
  void set_bike_tag(const bool bike_tag) {
    attributes_.fields.bike_tag = bike_tag;
  }

  /**
   * Get the bike_tag flag.
   * @return  Returns bike_tag flag.
   */
  bool bike_tag() const {
    return attributes_.fields.bike_tag;
  }

  /**
   * Sets the moped_tag flag.
   * @param  moped_tag  Mopeds/Electric scooters allowed on this way?
   */
  void set_moped_tag(const bool moped_tag) {
    attributes_.fields.moped_tag = moped_tag;
  }

  /**
   * Get the moped_tag flag.
   * @return  Returns moped_tag flag.
   */
  bool moped_tag() const {
    return attributes_.fields.moped_tag;
  }

  /**
   * Sets the bus_tag flag.
   * @param  bus_tag    Buses allowed on this way?
   */
  void set_bus_tag(const bool bus_tag) {
    attributes_.fields.bus_tag = bus_tag;
  }

  /**
   * Get the bus_tag flag.
   * @return  Returns bus_tag flag.
   */
  bool bus_tag() const {
    return attributes_.fields.bus_tag;
  }

  /**
   * Sets the foot_tag flag.
   * @param  foot_tag   Pedestrians allowed on this way?
   */
  void set_foot_tag(const bool foot_tag) {
    attributes_.fields.foot_tag = foot_tag;
  }

  /**
   * Get the foot_tag flag.
   * @return  Returns foot_tag flag.
   */
  bool foot_tag() const {
    return attributes_.fields.foot_tag;
  }

  /**
   * Sets the truck_tag flag.
   * @param  truck_tag    Trucks allowed on this way?
   */
  void set_truck_tag(const bool truck_tag) {
    attributes_.fields.truck_tag = truck_tag;
  }

  /**
   * Get the truck_tag flag.
   * @return  Returns truck_tag flag.
   */
  bool truck_tag() const {
    return attributes_.fields.truck_tag;
  }

  /**
   * Sets the hov_tag flag.
   * @param  hov_tag    hov way?
   */
  void set_hov_tag(const bool hov_tag) {
    attributes_.fields.hov_tag = hov_tag;
  }

  /**
   * Get the hov_tag flag.
   * @return  Returns hov_tag flag.
   */
  bool hov_tag() const {
    return attributes_.fields.hov_tag;
  }

  /**
   * Sets the taxi_tag flag.
   * @param  taxi_tag
   */
  void set_taxi_tag(const bool taxi_tag) {
    attributes_.fields.taxi_tag = taxi_tag;
  }

  /**
   * Get the taxi_tag flag.
   * @return  Returns taxi_tag flag.
   */
  bool taxi_tag() const {
    return attributes_.fields.taxi_tag;
  }

  /**
   * Sets the motorroad_tag flag.
   * @param  motorroad_tag    motorroad tag exists?
   */
  void set_motorroad_tag(const bool motorroad_tag) {
    attributes_.fields.motorroad_tag = motorroad_tag;
  }

  /**
   * Get the motorroad_tag flag.
   * @return  Returns motorroad_tag flag.
   */
  bool motorroad_tag() const {
    return attributes_.fields.motorroad_tag;
  }

  /**
   * Sets the motorcycle_tag flag.
   * @param  motorcycle_tag    motorcycle tag exists?
   */
  void set_motorcycle_tag(const bool motorcycle_tag) {
    attributes_.fields.motorcycle_tag = motorcycle_tag;
  }

  /**
   * Get the motorcycle_tag flag.
   * @return  Returns motorcycle_tag flag.
   */
  bool motorcycle_tag() const {
    return attributes_.fields.motorcycle_tag;
  }

  // OSM way Id
  uint64_t osmwayid_;

  // Access attributes
  union AccessAttributes {
    struct Fields {
      uint16_t auto_tag : 1;
      uint16_t bike_tag : 1;
      uint16_t bus_tag : 1;
      uint16_t foot_tag : 1;
      uint16_t truck_tag : 1;
      uint16_t hov_tag : 1;
      uint16_t taxi_tag : 1;
      uint16_t motorroad_tag : 1;
      uint16_t moped_tag : 1;
      uint16_t motorcycle_tag : 1;
      uint16_t spare : 6;
    } fields;
    uint32_t v; // this should be 64bits wide for architectures who require word alignment
  };
  AccessAttributes attributes_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMACCESS_H
