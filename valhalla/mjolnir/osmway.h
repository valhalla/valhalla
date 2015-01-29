#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// OSM way
class OSMWay {
 public:

  /**
   * Constructor
   */
  OSMWay();

  /**
   * Constructor given a way id.
   * @param   id  way id
   */
  OSMWay(uint64_t id);

  /**
   * Destructor
   */
  ~OSMWay();

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
   * Set local nodelist.
   * @param   nodes  list of nodes for this way
   */
  void set_nodes(const std::vector<uint64_t> &nodes);
  /**
   * Get the number of nodes for this way.
   */
  uint32_t node_count() const;

  /**
   * Get the list of nodes for this way.
   * @return  Returns nodes
   */
  const std::vector<uint64_t>& nodes() const;

  /**
   * Sets the speed
   * @param  speed   Speed in KPH.
   */
  void set_speed(const float speed);

  /**
   * Gets the speed in KPH.
   * @return  Returns speed.
   */
  float speed() const;

  /**
   * Sets the ref
   * @param  ref   Reference.
   */
  void set_ref(const std::string& ref);

  /**
   * Get the ref.
   * @return  Returns ref.
   */
  const std::string& ref() const;

  /**
   * Sets the int ret
   * @param  int_ref   International reference.
   */
  void set_int_ref(const std::string& int_ref);

  /**
   * Get the int ref.
   * @return  Returns int ref.
   */
  const std::string& int_ref() const;

  /**
   * Sets the name
   * @param  name   Name.
   */
  void set_name(const std::string& name);

  /**
   * Get the name.
   * @return  Returns name.
   */
  const std::string& name() const;

  /**
   * Sets the name:en
   * @param  name:en   English name.
   */
  void set_name_en(const std::string& name_en);

  /**
   * Get the name:en.
   * @return  Returns english name.
   */
  const std::string& name_en() const;

  /**
   * Sets the alt name
   * @param  alt_name   Alt name.
   */
  void set_alt_name(const std::string& alt_name);

  /**
   * Get the alt name.
   * @return  Returns alt name.
   */
  const std::string& alt_name() const;

  /**
   * Sets the official name
   * @param  official_name   Official name.
   */
  void set_official_name(const std::string& official_name);

  /**
   * Get the official name.
   * @return  Returns official name.
   */
  const std::string& official_name() const;

  /**
   * Sets the destination.
   * @param  destination   Destination.
   */
  void set_destination(const std::string& destination);

  /**
   * Get the get_destination.
   * @return  Returns destination.
   */
  const std::string& destination() const;

  /**
   * Sets the destination ref.
   * @param  destination_ref   Destination ref.
   */
  void set_destination_ref(const std::string& destination_ref);

  /**
   * Get the destination_ref.
   * @return  Returns destination ref.
   */
  const std::string& destination_ref() const;

  /**
   * Sets the destination ref to.
   * @param  destination_ref_to   Destination ref to.
   */
  void set_destination_ref_to(const std::string& destination_ref_to);

  /**
   * Get the destination ref to.
   * @return  Returns destination ref to.
   */
  const std::string& destination_ref_to() const;

  /**
   * Sets the junction ref.
   * @param  junction_ref   Junction ref.
   */
  void set_junction_ref(const std::string& junction_ref);

  /**
   * Get the junction ref.
   * @return  Returns junction ref.
   */
  const std::string& junction_ref() const;

  /**
   * Sets the bike national ref.
   * @param  bike_national_ref   Name of the national bike network.
   */
  void set_bike_national_ref(const std::string& bike_national_ref);

  /**
   * Get the bike national ref.
   * @return  Returns national bike network name.
   */
  const std::string& bike_national_ref() const;

  /**
   * Sets the bike regional ref.
   * @param  bike_regional_ref   Name of the regional bike network.
   */
  void set_bike_regional_ref(const std::string& bike_regional_ref);

  /**
   * Get the bike regional ref.
   * @return  Returns regional bike network name.
   */
  const std::string& bike_regional_ref() const;

  /**
   * Sets the bike local ref.
   * @param  bike_local_ref   Name of the local bike network.
   */
  void set_bike_local_ref(const std::string& bike_local_ref);

  /**
   * Get the bike local ref.
   * @return  Returns local bike network name.
   */
  const std::string& bike_local_ref() const;

  /**
   * Sets the auto_forward flag.
   * @param  auto_forward   Can you drive in the forward direction?
   */
  void set_auto_forward(const bool auto_forward);

  /**
   * Get the auto forward flag.
   * @return  Returns auto forward flag.
   */
  bool auto_forward() const;

  /**
   * Sets the bike_forward flag.
   * @param  bike_forward   Can you bike in the forward direction?
   */
  void set_bike_forward(const bool bike_forward);

  /**
   * Get the bike forward flag.
   * @return  Returns bike forward flag.
   */
  bool bike_forward() const;

  /**
   * Sets the auto_backward flag.
   * @param  auto_backward   Can you drive in the reverse direction?
   */
  void set_auto_backward(const bool auto_backward);

  /**
   * Get the auto backward flag.
   * @return  Returns auto backward flag.
   */
  bool auto_backward() const;

  /**
   * Sets the bike_backward flag.
   * @param  bike_backward   Can you bike in the reverse direction?
   */
  void set_bike_backward(const bool bike_backward);

  /**
   * Get the bike backward flag.
   * @return  Returns bike backward flag.
   */
  bool bike_backward() const;

  /**
   * Sets the destination_only flag.
   * @param  destination_only   Is private?
   */
  void set_destination_only(const bool destination_only);

  /**
   * Get the destination only/private flag.
   * @return  Returns private flag.
   */
  bool destination_only() const;

  /**
   * Sets the pedestrian flag.
   * @param  pedestrian   Are pedestrians allowed?
   */
  void set_pedestrian(const bool pedestrian);

  /**
   * Get the pedestrian flag.
   * @return  Returns pedestrian flag.
   */
  bool pedestrian() const;

  /**
   * Sets the no thru traffic flag.
   * @param  no_thru_traffic   Traffic allowed?
   */
  void set_no_thru_traffic(const bool no_thru_traffic);

  /**
   * Get the no thru traffic flag.
   * @return  Returns no thru traffic flag.
   */
  bool no_thru_traffic() const;

  /**
   * Sets the oneway flag.
   * @param  oneway   Is oneway?
   */
  void set_oneway(const bool oneway);

  /**
   * Get the oneway flag.
   * @return  Returns oneway flag.
   */
  bool oneway() const;

  /**
   * Sets the roundabout flag.
   * @param  roundabout   Is a roundabout?
   */
  void set_roundabout(const bool roundabout);

  /**
   * Get the roundabout flag.
   * @return  Returns roundabout flag.
   */
  bool roundabout() const;

  /**
   * Sets the ferry flag.
   * @param  ferry   Is a ferry?
   */
  void set_ferry(const bool ferry);

  /**
   * Get the ferry flag.
   * @return  Returns ferry flag.
   */
  bool ferry() const;

  /**
   * Sets the rail flag.
   * @param  rail   Is a auto train?
   */
  void set_rail(const bool rail);

  /**
   * Get the rail flag.
   * @return  Returns rail flag.
   */
  bool rail() const;

  /**
   * Sets the surface.
   * @param  surface
   */
  void set_surface(const Surface surface);

  /**
   * Get the surface.
   * @return  Returns Surface.
   */
  Surface surface() const;

  /**
   * Sets the cycle lane.
   * @param  cyclelane
   */
  void set_cyclelane(const CycleLane cyclelane);

  /**
   * Get the cycle lane.
   * @return  Returns CycleLane.
   */
  CycleLane cyclelane() const;

  /**
   * Sets the number of lanes
   * @param  lanes  Number of lanes
   */
  void set_lanes(const uint32_t lanes);

  /**
   * Get the number of lanes
   * @return  Returns number of lanes.
   */
  uint32_t lanes() const;

  /**
   * Sets the tunnel flag.
   * @param  tunnel   Is a tunnel road?
   */
  void set_tunnel(const bool tunnel);

  /**
   * Get the tunnel flag.
   * @return  Returns tunnel flag.
   */
  bool tunnel() const;

  /**
   * Sets the toll flag.
   * @param  toll   Is a toll road?
   */
  void set_toll(const bool toll);

  /**
   * Get the toll flag.
   * @return  Returns toll flag.
   */
  bool toll() const;

  /**
   * Sets the bridge flag.
   * @param  bridge   Is a bridge?
   */
  void set_bridge(const bool bridge);

  /**
   * Get the bridge flag.
   * @return  Returns bridge flag.
   */
  bool bridge() const;

  /**
   * Sets the bike network.
   * @param  bike_network Mask of the bike networks (ncn/rcn/lcn).
   */
  void set_bike_network(const uint32_t bike_network);

  /**
   * Get the bike network mask.
   * @return  Returns the bike network mask.
   */
  uint32_t bike_network() const;

  /**
   * Sets the exit tag.
   * @param  exit       Exit flag.
   */
  void set_exit(const bool exit);

  /**
   * Get the exit flag.
   * @return  Returns exit flag.
   */
  bool exit() const;

  /**
   * Get the road class.
   * @return  Returns road class.
   */
  RoadClass road_class() const;

  /**
   * Sets the road class.
   * @param  roadclass  Road Class/highway type.
   */
  void set_road_class(const RoadClass roadclass);

  /**
   * Sets the use tag.
   * @param  use       use. None Cycleway ParkingAisle, Driveway, Alley,
   *                        EmergencyAccess, DriveThru, Steps, and Other
   */
  void set_use(const Use use);

  /**
   * Get the use.
   * @return  Returns use.
   */
  Use use() const;

  /**
   * Sets the link tag.
   * @param  link       Link.  Ramp or turn channel.
   */
  void set_link(const bool link);

  /**
   * Get the link flag.
   * @return  Returns link flag.
   */
  bool link() const;

  /**
   * Get the names for the edge info based on the road class.
   * @return  Returns vector of strings
   */
  std::vector<std::string> GetNames() const;

 private:
  // OSM way Id
  uint64_t osmwayid_;

  // List of OSM node Ids along the way
  std::vector<uint64_t> nodes_;

  // Reference name (highway numbers)
  std::string ref_;
  std::string int_ref_;

  // Names
  std::string name_;
  std::string name_en_;
  std::string alt_name_;
  std::string official_name_;

  // Sign Destination information
  std::string destination_;
  std::string destination_ref_;
  std::string destination_ref_to_;
  std::string junction_ref_;

  // Bike network information
  std::string bike_national_ref_;
  std::string bike_regional_ref_;
  std::string bike_local_ref_;

  // Way attributes
  union WayAttributes {
    struct Fields {
      uint32_t auto_forward     :1;
      uint32_t bike_forward     :1;
      uint32_t auto_backward    :1;
      uint32_t bike_backward    :1;
      uint32_t pedestrian       :1;
      uint32_t destination_only :1;
      uint32_t no_thru_traffic  :1;
      uint32_t oneway           :1;
      uint32_t roundabout       :1;
      uint32_t ferry            :1;
      uint32_t rail             :1;
      uint32_t surface          :3;
      uint32_t cycle_lane       :2;
      uint32_t lanes            :4;
      uint32_t tunnel           :1;
      uint32_t toll             :1;
      uint32_t bridge           :1;
      uint32_t bikenetwork      :4;
      uint32_t exit             :1;
      uint32_t spare            :4;
    } fields;
    uint32_t v;
  };
  WayAttributes attributes_;

  union Classification {
    struct Fields {
      uint8_t road_class        :3;     // Importance of the road/path
      uint8_t link              :1;     // *link tag - Ramp or turn channel
      uint8_t use               :4;     // Use / form
    } fields;
    uint8_t v;
  };
  Classification classification_;

  // Speed in kilometers per hour
  uint8_t speed_;
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
