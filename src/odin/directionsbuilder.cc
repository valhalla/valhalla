#include <iostream>
#include <unordered_map>

#include "exception.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/maneuversbuilder.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrativebuilder.h"

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/directions.pb.h>

namespace {
// Minimum edge length (~10 feet)
constexpr auto kMinEdgeLength = 0.003f;

} // namespace

namespace valhalla {
namespace odin {

// Note: reason for casting the Enum to int here:
// https://stackoverflow.com/questions/18837857/cant-use-enum-class-as-unordered-map-key
const std::unordered_map<int, TripDirections_VehicleType> translate_vehicle_type{
    {static_cast<int>(TripPath_VehicleType_kCar), TripDirections_VehicleType_kCar},
    {static_cast<int>(TripPath_VehicleType_kMotorcycle), TripDirections_VehicleType_kMotorcycle},
    {static_cast<int>(TripPath_VehicleType_kAutoBus), TripDirections_VehicleType_kAutoBus},
    {static_cast<int>(TripPath_VehicleType_kTractorTrailer),
     TripDirections_VehicleType_kTractorTrailer},
    {static_cast<int>(TripPath_VehicleType_kMotorScooter), TripDirections_VehicleType_kMotorScooter},
};

const std::unordered_map<int, TripDirections_PedestrianType> translate_pedestrian_type{
    {static_cast<int>(TripPath_PedestrianType_kFoot), TripDirections_PedestrianType_kFoot},
    {static_cast<int>(TripPath_PedestrianType_kWheelchair),
     TripDirections_PedestrianType_kWheelchair},
    {static_cast<int>(TripPath_PedestrianType_kSegway), TripDirections_PedestrianType_kSegway},
};

const std::unordered_map<int, TripDirections_BicycleType> translate_bicycle_type{
    {static_cast<int>(TripPath_BicycleType_kRoad), TripDirections_BicycleType_kRoad},
    {static_cast<int>(TripPath_BicycleType_kCross), TripDirections_BicycleType_kCross},
    {static_cast<int>(TripPath_BicycleType_kHybrid), TripDirections_BicycleType_kHybrid},
    {static_cast<int>(TripPath_BicycleType_kMountain), TripDirections_BicycleType_kMountain},
};

const std::unordered_map<int, TripDirections_TransitType> translate_transit_type{
    {static_cast<int>(TripPath_TransitType_kTram), TripDirections_TransitType_kTram},
    {static_cast<int>(TripPath_TransitType_kMetro), TripDirections_TransitType_kMetro},
    {static_cast<int>(TripPath_TransitType_kRail), TripDirections_TransitType_kRail},
    {static_cast<int>(TripPath_TransitType_kBus), TripDirections_TransitType_kBus},
    {static_cast<int>(TripPath_TransitType_kFerry), TripDirections_TransitType_kFerry},
    {static_cast<int>(TripPath_TransitType_kCableCar), TripDirections_TransitType_kCableCar},
    {static_cast<int>(TripPath_TransitType_kGondola), TripDirections_TransitType_kGondola},
    {static_cast<int>(TripPath_TransitType_kFunicular), TripDirections_TransitType_kFunicular},
};

const std::unordered_map<int, TripDirections_TravelMode> translate_travel_mode{
    {static_cast<int>(TripPath_TravelMode_kDrive), TripDirections_TravelMode_kDrive},
    {static_cast<int>(TripPath_TravelMode_kPedestrian), TripDirections_TravelMode_kPedestrian},
    {static_cast<int>(TripPath_TravelMode_kBicycle), TripDirections_TravelMode_kBicycle},
    {static_cast<int>(TripPath_TravelMode_kTransit), TripDirections_TravelMode_kTransit},
};

const std::unordered_map<int, std::string> stringify_vehicle_type{
    {static_cast<int>(TripPath_VehicleType_kCar), "car"},
    {static_cast<int>(TripPath_VehicleType_kMotorcycle), "motorcycle"},
    {static_cast<int>(TripPath_VehicleType_kAutoBus), "autobus"},
    {static_cast<int>(TripPath_VehicleType_kTractorTrailer), "trailer"},
    {static_cast<int>(TripPath_VehicleType_kMotorScooter), "motorscooter"},
};

const std::unordered_map<int, std::string> stringify_pedestrian_type{
    {static_cast<int>(TripPath_PedestrianType_kFoot), "foot"},
    {static_cast<int>(TripPath_PedestrianType_kWheelchair), "wheelchair"},
    {static_cast<int>(TripPath_PedestrianType_kSegway), "segway"},
};

const std::unordered_map<int, std::string> stringify_bicycle_type{
    {static_cast<int>(TripPath_BicycleType_kRoad), "roadbike"},
    {static_cast<int>(TripPath_BicycleType_kCross), "crossbike"},
    {static_cast<int>(TripPath_BicycleType_kHybrid), "hybridbike"},
    {static_cast<int>(TripPath_BicycleType_kMountain), "mountainbike"},
};

const std::unordered_map<int, std::string> stringify_transit_type{
    {static_cast<int>(TripPath_TransitType_kTram), "tram"},
    {static_cast<int>(TripPath_TransitType_kMetro), "metro"},
    {static_cast<int>(TripPath_TransitType_kRail), "rail"},
    {static_cast<int>(TripPath_TransitType_kBus), "bus"},
    {static_cast<int>(TripPath_TransitType_kFerry), "ferry"},
    {static_cast<int>(TripPath_TransitType_kCableCar), "cablecar"},
    {static_cast<int>(TripPath_TransitType_kGondola), "gondola"},
    {static_cast<int>(TripPath_TransitType_kFunicular), "funicular"},
};

const std::unordered_map<int, std::string> stringify_travel_mode{
    {static_cast<int>(TripDirections_TravelMode_kDrive), "driving"},
    {static_cast<int>(TripDirections_TravelMode_kPedestrian), "walking"},
    {static_cast<int>(TripDirections_TravelMode_kBicycle), "cycling"},
    {static_cast<int>(TripDirections_TravelMode_kTransit), "transit"},
};

const std::unordered_map<int, std::string> stringify_maneuver_type {
    {static_cast<int>(TripDirections_Maneuver_Type_kNone), "none"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStart), "start"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStartRight), "start_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStartLeft), "start_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kDestination), "destination"},
    {static_cast<int>(TripDirections_Maneuver_Type_kDestinationRight), "destination_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kDestinationLeft), "destination_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kBecomes), "becomes"},
    {static_cast<int>(TripDirections_Maneuver_Type_kContinue), "continue"},
    {static_cast<int>(TripDirections_Maneuver_Type_kSlightRight), "slight_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRight), "right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kSharpRight), "sharp_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kUturnRight), "u-turn-right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kUturnLeft), "turn_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kSharpLeft), "sharp_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kLeft), "left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kSlightLeft), "slight_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRampStraight), "ramp_straight"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRampRight), "ramp_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRampLeft), "ramp_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kExitRight), "exit_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kExitLeft), "exit_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStayStraight), "stay_straight"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStayRight), "stay_right"},
    {static_cast<int>(TripDirections_Maneuver_Type_kStayLeft), "stay_left"},
    {static_cast<int>(TripDirections_Maneuver_Type_kMerge), "merge"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRoundaboutEnter), "roundabout_enter"},
    {static_cast<int>(TripDirections_Maneuver_Type_kRoundaboutExit), "roundabout_exit"},
    {static_cast<int>(TripDirections_Maneuver_Type_kFerryEnter), "ferry_enter"},
    {static_cast<int>(TripDirections_Maneuver_Type_kFerryExit), "ferry_exit"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransit), "transit"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransitTransfer), "transit_transfer"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransitRemainOn), "transit_remain_on"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransitConnectionStart), "transit_connection_start"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransitConnectionTransfer), "transit_connection_transfer"},
    {static_cast<int>(TripDirections_Maneuver_Type_kTransitConnectionDestination), "transit_connection_destination"},
    {static_cast<int>(TripDirections_Maneuver_Type_kPostTransitConnectionDestination), "post_transit_connection_destination"}
};


DirectionsBuilder::DirectionsBuilder() {
}

// Returns the trip directions based on the specified directions options
// and trip path. This method calls ManeuversBuilder::Build and
// NarrativeBuilder::Build to form the maneuver list. This method
// calls PopulateTripDirections to transform the maneuver list into the
// trip directions.
TripDirections DirectionsBuilder::Build(const DirectionsOptions& directions_options,
                                        TripPath& trip_path) {
  // Validate trip path node list
  if (trip_path.node_size() < 1) {
    throw valhalla_exception_t{210};
  }

  EnhancedTripPath* etp = static_cast<EnhancedTripPath*>(&trip_path);

  // Produce maneuvers and narrative if enabled
  std::list<Maneuver> maneuvers;
  if (directions_options.narrative()) {
    // Update the heading of ~0 length edges
    UpdateHeading(etp);

    // Create maneuvers
    ManeuversBuilder maneuversBuilder(directions_options, etp);
    maneuvers = maneuversBuilder.Build();

    // Create the narrative
    std::unique_ptr<NarrativeBuilder> narrative_builder =
        NarrativeBuilderFactory::Create(directions_options, etp);
    narrative_builder->Build(directions_options, etp, maneuvers);
  }

  // Return trip directions
  return PopulateTripDirections(directions_options, etp, maneuvers);
}


proto::Directions DirectionsBuilder::BuildProto(const DirectionsOptions& directions_options,
                                        std::list<TripPath>& legs) {

  proto::Directions proto_directions;


  proto::Route* proto_route = proto_directions.add_routes(); // TODO: do this for every alternative route

  proto_route->set_distance(0);
  proto_route->set_duration(0);
  proto_route->mutable_bounding_box()->set_min_lat(90);
  proto_route->mutable_bounding_box()->set_min_lon(180);
  proto_route->mutable_bounding_box()->set_max_lat(-90);
  proto_route->mutable_bounding_box()->set_max_lon(-180);


  for (auto& leg : legs) {
    // Validate trip path node list
    if (leg.node_size() < 1) {
      throw valhalla_exception_t{210};
    }

    EnhancedTripPath* etp = static_cast<EnhancedTripPath*>(&leg);

    // Produce maneuvers and narrative if enabled
    std::list<Maneuver> maneuvers;
    if (directions_options.narrative()) {
      // Update the heading of ~0 length edges
      UpdateHeading(etp);

      // Create maneuvers
      ManeuversBuilder maneuversBuilder(directions_options, etp);
      maneuvers = maneuversBuilder.Build();

      // Create the narrative
      std::unique_ptr<NarrativeBuilder> narrative_builder =
          NarrativeBuilderFactory::Create(directions_options, etp);
      narrative_builder->Build(directions_options, etp, maneuvers);
    }
    proto::Leg* proto_leg = proto_route->add_legs();
    PopulateRouteLegProto(directions_options, etp, maneuvers, proto_leg);

    proto_route->set_distance(proto_route->distance() + proto_leg->distance());
    proto_route->set_duration(proto_route->duration() + proto_leg->duration());
    proto_route->mutable_bounding_box()->set_min_lat(std::min(proto_route->mutable_bounding_box()->min_lat(), proto_leg->mutable_bounding_box()->min_lat()));
    proto_route->mutable_bounding_box()->set_min_lon(std::min(proto_route->mutable_bounding_box()->min_lon(), proto_leg->mutable_bounding_box()->min_lon()));
    proto_route->mutable_bounding_box()->set_max_lat(std::max(proto_route->mutable_bounding_box()->max_lat(), proto_leg->mutable_bounding_box()->max_lat()));
    proto_route->mutable_bounding_box()->set_max_lon(std::max(proto_route->mutable_bounding_box()->max_lon(), proto_leg->mutable_bounding_box()->max_lon()));
  }

  return proto_directions;
}



// Update the heading of ~0 length edges.
void DirectionsBuilder::UpdateHeading(EnhancedTripPath* etp) {
  for (size_t x = 0; x < etp->node_size(); ++x) {
    auto* prev_edge = etp->GetPrevEdge(x);
    auto* curr_edge = etp->GetCurrEdge(x);
    auto* next_edge = etp->GetNextEdge(x);
    if (curr_edge && (curr_edge->length() < kMinEdgeLength)) {

      // Set the current begin heading
      if (prev_edge && (prev_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_begin_heading(prev_edge->end_heading());
      } else if (next_edge && (next_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_begin_heading(next_edge->begin_heading());
      }

      // Set the current end heading
      if (next_edge && (next_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_end_heading(next_edge->begin_heading());
      } else if (prev_edge && (prev_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_end_heading(prev_edge->end_heading());
      }
    }
  }
}

// Returns the trip directions based on the specified directions options,
// trip path, and maneuver list.
TripDirections DirectionsBuilder::PopulateTripDirections(const DirectionsOptions& directions_options,
                                                         EnhancedTripPath* etp,
                                                         std::list<Maneuver>& maneuvers) {
  TripDirections trip_directions;

  // Populate trip and leg IDs
  trip_directions.set_trip_id(etp->trip_id());
  trip_directions.set_leg_id(etp->leg_id());
  trip_directions.set_leg_count(etp->leg_count());

  // Populate locations
  trip_directions.mutable_location()->CopyFrom(etp->location());

  // Populate maneuvers
  for (const auto& maneuver : maneuvers) {
    auto* trip_maneuver = trip_directions.add_maneuver();
    trip_maneuver->set_type(maneuver.type());
    trip_maneuver->set_text_instruction(maneuver.instruction());
    trip_maneuver->set_begin_path_index(maneuver.begin_node_index());
    trip_maneuver->set_end_path_index(maneuver.end_node_index());

    // Set street names
    for (const auto& street_name : maneuver.street_names()) {
      trip_maneuver->add_street_name(street_name->value());
    }

    // Set begin street names
    for (const auto& begin_street_name : maneuver.begin_street_names()) {
      trip_maneuver->add_begin_street_name(begin_street_name->value());
    }

    trip_maneuver->set_length(maneuver.length(directions_options.units()));
    trip_maneuver->set_time(maneuver.time());
    trip_maneuver->set_begin_cardinal_direction(maneuver.begin_cardinal_direction());
    trip_maneuver->set_begin_heading(maneuver.begin_heading());
    trip_maneuver->set_begin_shape_index(maneuver.begin_shape_index());
    trip_maneuver->set_end_shape_index(maneuver.end_shape_index());
    if (maneuver.portions_toll()) {
      trip_maneuver->set_portions_toll(maneuver.portions_toll());
    }
    if (maneuver.portions_unpaved()) {
      trip_maneuver->set_portions_unpaved(maneuver.portions_unpaved());
    }

    if (maneuver.HasVerbalTransitionAlertInstruction()) {
      trip_maneuver->set_verbal_transition_alert_instruction(
          maneuver.verbal_transition_alert_instruction());
    }

    if (maneuver.HasVerbalPreTransitionInstruction()) {
      trip_maneuver->set_verbal_pre_transition_instruction(
          maneuver.verbal_pre_transition_instruction());
    }

    if (maneuver.HasVerbalPostTransitionInstruction()) {
      trip_maneuver->set_verbal_post_transition_instruction(
          maneuver.verbal_post_transition_instruction());
    }

    // Populate sign information
    if (maneuver.HasExitSign()) {
      auto* trip_sign = trip_maneuver->mutable_sign();

      // Process exit number info
      if (maneuver.HasExitNumberSign()) {
        auto* trip_exit_number_elements = trip_sign->mutable_exit_number_elements();
        for (const auto& exit_number : maneuver.signs().exit_number_list()) {
          auto* trip_exit_number_element = trip_exit_number_elements->Add();
          trip_exit_number_element->set_text(exit_number.text());
          trip_exit_number_element->set_consecutive_count(exit_number.consecutive_count());
        }
      }

      // Process exit branch info
      if (maneuver.HasExitBranchSign()) {
        auto* trip_exit_branch_elements = trip_sign->mutable_exit_branch_elements();
        for (const auto& exit_branch : maneuver.signs().exit_branch_list()) {
          auto* trip_exit_branch_element = trip_exit_branch_elements->Add();
          trip_exit_branch_element->set_text(exit_branch.text());
          trip_exit_branch_element->set_consecutive_count(exit_branch.consecutive_count());
        }
      }

      // Process exit toward info
      if (maneuver.HasExitTowardSign()) {
        auto* trip_exit_toward_elements = trip_sign->mutable_exit_toward_elements();
        for (const auto& exit_toward : maneuver.signs().exit_toward_list()) {
          auto* trip_exit_toward_element = trip_exit_toward_elements->Add();
          trip_exit_toward_element->set_text(exit_toward.text());
          trip_exit_toward_element->set_consecutive_count(exit_toward.consecutive_count());
        }
      }

      // Process exit name info
      if (maneuver.HasExitNameSign()) {
        auto* trip_exit_name_elements = trip_sign->mutable_exit_name_elements();
        for (const auto& exit_name : maneuver.signs().exit_name_list()) {
          auto* trip_exit_name_element = trip_exit_name_elements->Add();
          trip_exit_name_element->set_text(exit_name.text());
          trip_exit_name_element->set_consecutive_count(exit_name.consecutive_count());
        }
      }
    }

    // Roundabout exit count
    if (maneuver.roundabout_exit_count() > 0) {
      trip_maneuver->set_roundabout_exit_count(maneuver.roundabout_exit_count());
    }

    // Depart instructions
    if (!maneuver.depart_instruction().empty()) {
      trip_maneuver->set_depart_instruction(maneuver.depart_instruction());
    }
    if (!maneuver.verbal_depart_instruction().empty()) {
      trip_maneuver->set_verbal_depart_instruction(maneuver.verbal_depart_instruction());
    }

    // Arrive instructions
    if (!maneuver.arrive_instruction().empty()) {
      trip_maneuver->set_arrive_instruction(maneuver.arrive_instruction());
    }
    if (!maneuver.verbal_arrive_instruction().empty()) {
      trip_maneuver->set_verbal_arrive_instruction(maneuver.verbal_arrive_instruction());
    }

    // Process transit route
    if (maneuver.IsTransit()) {
      const auto& transit_route = maneuver.transit_info();
      auto* trip_transit_info = trip_maneuver->mutable_transit_info();
      if (!transit_route.onestop_id.empty()) {
        trip_transit_info->set_onestop_id(transit_route.onestop_id);
      }
      if (!transit_route.short_name.empty()) {
        trip_transit_info->set_short_name(transit_route.short_name);
      }
      if (!transit_route.long_name.empty()) {
        trip_transit_info->set_long_name(transit_route.long_name);
      }
      if (!transit_route.headsign.empty()) {
        trip_transit_info->set_headsign(transit_route.headsign);
      }
      trip_transit_info->set_color(transit_route.color);
      trip_transit_info->set_text_color(transit_route.text_color);
      if (!transit_route.description.empty()) {
        trip_transit_info->set_description(transit_route.description);
      }
      if (!transit_route.operator_onestop_id.empty()) {
        trip_transit_info->set_operator_onestop_id(transit_route.operator_onestop_id);
      }
      if (!transit_route.operator_name.empty()) {
        trip_transit_info->set_operator_name(transit_route.operator_name);
      }
      if (!transit_route.operator_url.empty()) {
        trip_transit_info->set_operator_url(transit_route.operator_url);
      }

      // Process transit stops
      for (const auto& transit_platform : transit_route.transit_stops) {
        trip_transit_info->add_transit_stops()->CopyFrom(transit_platform);
      }
    }

    // Verbal multi-cue
    if (maneuver.verbal_multi_cue()) {
      trip_maneuver->set_verbal_multi_cue(maneuver.verbal_multi_cue());
    }

    // Travel mode
    trip_maneuver->set_travel_mode(translate_travel_mode.find(maneuver.travel_mode())->second);

    // Travel type
    switch (maneuver.travel_mode()) {
      case TripPath_TravelMode_kDrive: {
        trip_maneuver->set_vehicle_type(translate_vehicle_type.find(maneuver.vehicle_type())->second);
        break;
      }
      case TripPath_TravelMode_kPedestrian: {
        trip_maneuver->set_pedestrian_type(
            translate_pedestrian_type.find(maneuver.pedestrian_type())->second);
        break;
      }
      case TripPath_TravelMode_kBicycle: {
        trip_maneuver->set_bicycle_type(translate_bicycle_type.find(maneuver.bicycle_type())->second);
        break;
      }
      case TripPath_TravelMode_kTransit: {
        trip_maneuver->set_transit_type(translate_transit_type.find(maneuver.transit_type())->second);
        break;
      }
    }
  }

  // Populate summary
  trip_directions.mutable_summary()->set_length(etp->GetLength(directions_options.units()));
  trip_directions.mutable_summary()->set_time(etp->node(etp->GetLastNodeIndex()).elapsed_time());
  auto mutable_bbox = trip_directions.mutable_summary()->mutable_bbox();
  mutable_bbox->mutable_min_ll()->set_lat(etp->bbox().min_ll().lat());
  mutable_bbox->mutable_min_ll()->set_lng(etp->bbox().min_ll().lng());
  mutable_bbox->mutable_max_ll()->set_lat(etp->bbox().max_ll().lat());
  mutable_bbox->mutable_max_ll()->set_lng(etp->bbox().max_ll().lng());

  // Populate shape
  trip_directions.set_shape(etp->shape());

  return trip_directions;
}


// Returns the trip directions based on the specified directions options,
// trip path, and maneuver list.
void DirectionsBuilder::PopulateRouteLegProto(const DirectionsOptions& directions_options,
                                                    EnhancedTripPath* etp,
                                                    std::list<Maneuver>& odin_maneuvers,
                                                    proto::Leg* proto_leg) {

  // proto_leg->set_summary("Route to " + odin_maneuvers.back().street_names().front()->value());  // TODO: write better summaries
  proto_leg->set_distance(etp->GetLength(directions_options.units()));  // TODO: check distance with units
  proto_leg->set_duration(etp->node(etp->GetLastNodeIndex()).elapsed_time());
  proto_leg->mutable_bounding_box()->set_min_lat(etp->bbox().min_ll().lat());
  proto_leg->mutable_bounding_box()->set_min_lon(etp->bbox().min_ll().lng());
  proto_leg->mutable_bounding_box()->set_max_lat(etp->bbox().max_ll().lat());
  proto_leg->mutable_bounding_box()->set_max_lon(etp->bbox().max_ll().lng());
  proto_leg->set_geometry(etp->shape());


  std::size_t maneuver_n = 0;
  for (const auto& odin_maneuver : odin_maneuvers) {

    if (odin_maneuver.type() != TripDirections_Maneuver_Type_kDestination &&
        odin_maneuver.type() != TripDirections_Maneuver_Type_kDestinationRight &&
        odin_maneuver.type() != TripDirections_Maneuver_Type_kDestinationLeft) {
      // std::cout << "Creating Step #" << maneuver_n << " of type " << TripDirections_Maneuver_Type_Name(odin_maneuver.type()) << std::endl;
      proto::Step* proto_step = proto_leg->add_steps();

      proto_step->set_distance(odin_maneuver.length(directions_options.units()));
      proto_step->set_duration(odin_maneuver.time());
      proto_step->set_geometry_index_begin(odin_maneuver.begin_shape_index());
      proto_step->set_geometry_index_end(odin_maneuver.end_shape_index());
      proto_step->set_incoming_maneuver_index(maneuver_n);
      proto_step->set_outgoing_maneuver_index(maneuver_n + 1);
      if (!odin_maneuver.street_names().empty()) {
        proto_step->mutable_street()->set_name(odin_maneuver.street_names().front()->value()); // TODO fix this! see valhalla/issues/1367
        proto_step->mutable_street()->set_ref(odin_maneuver.street_names().back()->value());
      }
      // TODO: fix if find() doesn't find anything
      proto_step->set_travel_mode(stringify_travel_mode.find(odin_maneuver.travel_mode())->second);
      switch (odin_maneuver.travel_mode()) {
        case TripPath_TravelMode_kDrive: {
          proto_step->set_travel_mode_type(stringify_vehicle_type.find(odin_maneuver.vehicle_type())->second);
          break;
        }
        case TripPath_TravelMode_kPedestrian: {
          proto_step->set_travel_mode_type(
              stringify_pedestrian_type.find(odin_maneuver.pedestrian_type())->second);
          break;
        }
        case TripPath_TravelMode_kBicycle: {
          proto_step->set_travel_mode_type(stringify_bicycle_type.find(odin_maneuver.bicycle_type())->second);
          break;
        }
        case TripPath_TravelMode_kTransit: {
          proto_step->set_travel_mode_type(stringify_transit_type.find(odin_maneuver.transit_type())->second);
          break;
        }
      }
      proto_step->set_driving_side("right"); // TODO: fix this
    }

    // TODO repeated StreetName street_names = 5; // see valhalla/issues/1368
    // TODO optional Lane lane = 7;
    // TODO optional bool is_obvious = 8;

    proto::Maneuver* proto_maneuver = proto_leg->add_maneuvers();
    // TODO: fix/assert if find() doesn't find anything
    proto_maneuver->set_type(stringify_maneuver_type.find(odin_maneuver.type())->second);
    proto_maneuver->set_geometry_index(odin_maneuver.begin_shape_index());
    proto_maneuver->set_incoming_bearing(odin_maneuver.begin_heading()); // TODO: FIX THIS
    proto_maneuver->set_outgoing_bearing(odin_maneuver.begin_heading()); // TODO: FIX THIS is odin_maneuver.begin_heading() = outgoing_bearing?
    if (odin_maneuver.HasExitSign()) {  // TODO: What is consecutive_count used for?
      auto proto_maneuver_sign = proto_maneuver->mutable_sign();
      // TODO: Is this lambdaable?
      if (odin_maneuver.HasExitNumberSign()) {
        auto proto_maneuver_exit_sign_numbers = proto_maneuver_sign->mutable_exit_numbers();
        for (const auto& exit_number : odin_maneuver.signs().exit_number_list()) {
          auto proto_maneuver_exit_sign_number = proto_maneuver_exit_sign_numbers->Add();
          proto_maneuver_exit_sign_number->set_value(exit_number.text());
          proto_maneuver_exit_sign_number->set_consecutive_count(exit_number.consecutive_count());
        }
      }
      if (odin_maneuver.HasExitBranchSign()) {
        auto proto_maneuver_exit_sign_onto_streets = proto_maneuver_sign->mutable_exit_onto_streets();
        for (const auto& exit_branch : odin_maneuver.signs().exit_branch_list()) {
          auto proto_maneuver_exit_sign_onto_street = proto_maneuver_exit_sign_onto_streets->Add();
          proto_maneuver_exit_sign_onto_street->set_value(exit_branch.text());
          proto_maneuver_exit_sign_onto_street->set_consecutive_count(exit_branch.consecutive_count());
        }
      }
      if (odin_maneuver.HasExitTowardSign()) {
        auto proto_maneuver_exit_sign_toward_locations = proto_maneuver_sign->mutable_exit_toward_locations();
        for (const auto& exit_toward : odin_maneuver.signs().exit_toward_list()) {
          auto proto_maneuver_exit_sign_toward_location = proto_maneuver_exit_sign_toward_locations->Add();
          proto_maneuver_exit_sign_toward_location->set_value(exit_toward.text());
          proto_maneuver_exit_sign_toward_location->set_consecutive_count(exit_toward.consecutive_count());
        }
      }
      if (odin_maneuver.HasExitNameSign()) {
        auto proto_maneuver_exit_sign_exit_names = proto_maneuver_sign->mutable_exit_names();
        for (const auto& exit_name : odin_maneuver.signs().exit_name_list()) {
          auto proto_maneuver_exit_sign_exit_name = proto_maneuver_exit_sign_exit_names->Add();
          proto_maneuver_exit_sign_exit_name->set_value(exit_name.text());
          proto_maneuver_exit_sign_exit_name->set_consecutive_count(exit_name.consecutive_count());
        }
      }
      if (odin_maneuver.roundabout_exit_count() > 0) {
        proto_maneuver_sign->set_exit_roundabout_count(odin_maneuver.roundabout_exit_count());
      }
    }

    proto_maneuver->set_is_verbal_multi_cue(odin_maneuver.verbal_multi_cue());
    if (!odin_maneuver.depart_instruction().empty()) {
      proto_maneuver->set_text_instruction(odin_maneuver.depart_instruction());
    }
    else if (!odin_maneuver.arrive_instruction().empty()) {
      proto_maneuver->set_text_instruction(odin_maneuver.arrive_instruction());
    }
    if (odin_maneuver.HasVerbalTransitionAlertInstruction()) {
      proto_maneuver->set_verbal_transition_alert_instruction(
          odin_maneuver.verbal_transition_alert_instruction());
    }
    if (odin_maneuver.HasVerbalPreTransitionInstruction()) {
      proto_maneuver->set_verbal_pre_transition_instruction(
          odin_maneuver.verbal_pre_transition_instruction());
    }
    if (odin_maneuver.HasVerbalPostTransitionInstruction()) {
      proto_maneuver->set_verbal_post_transition_instruction(
          odin_maneuver.verbal_post_transition_instruction());
    }

    maneuver_n++;
  }
  assert(proto_leg->steps_size() + 1 == proto_leg->maneuvers_size());
}


} // namespace odin
} // namespace valhalla
