#include <iostream>
#include <unordered_map>

#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/maneuversbuilder.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrativebuilder.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "worker.h"

namespace {
// Minimum edge length to verify heading (~3 feet)
constexpr auto kMinEdgeLength = 0.001f;

} // namespace

namespace valhalla {
namespace odin {

const std::unordered_map<int, DirectionsLeg_VehicleType> translate_vehicle_type{
    {static_cast<int>(TripLeg_VehicleType_kCar), DirectionsLeg_VehicleType_kCar},
    {static_cast<int>(TripLeg_VehicleType_kMotorcycle), DirectionsLeg_VehicleType_kMotorcycle},
    {static_cast<int>(TripLeg_VehicleType_kAutoBus), DirectionsLeg_VehicleType_kAutoBus},
    {static_cast<int>(TripLeg_VehicleType_kTractorTrailer),
     DirectionsLeg_VehicleType_kTractorTrailer},
    {static_cast<int>(TripLeg_VehicleType_kMotorScooter), DirectionsLeg_VehicleType_kMotorScooter},
};

const std::unordered_map<int, DirectionsLeg_PedestrianType> translate_pedestrian_type{
    {static_cast<int>(TripLeg_PedestrianType_kFoot), DirectionsLeg_PedestrianType_kFoot},
    {static_cast<int>(TripLeg_PedestrianType_kWheelchair), DirectionsLeg_PedestrianType_kWheelchair},
    {static_cast<int>(TripLeg_PedestrianType_kSegway), DirectionsLeg_PedestrianType_kSegway},
};

const std::unordered_map<int, DirectionsLeg_BicycleType> translate_bicycle_type{
    {static_cast<int>(TripLeg_BicycleType_kRoad), DirectionsLeg_BicycleType_kRoad},
    {static_cast<int>(TripLeg_BicycleType_kCross), DirectionsLeg_BicycleType_kCross},
    {static_cast<int>(TripLeg_BicycleType_kHybrid), DirectionsLeg_BicycleType_kHybrid},
    {static_cast<int>(TripLeg_BicycleType_kMountain), DirectionsLeg_BicycleType_kMountain},
};

const std::unordered_map<int, DirectionsLeg_TransitType> translate_transit_type{
    {static_cast<int>(TripLeg_TransitType_kTram), DirectionsLeg_TransitType_kTram},
    {static_cast<int>(TripLeg_TransitType_kMetro), DirectionsLeg_TransitType_kMetro},
    {static_cast<int>(TripLeg_TransitType_kRail), DirectionsLeg_TransitType_kRail},
    {static_cast<int>(TripLeg_TransitType_kBus), DirectionsLeg_TransitType_kBus},
    {static_cast<int>(TripLeg_TransitType_kFerry), DirectionsLeg_TransitType_kFerry},
    {static_cast<int>(TripLeg_TransitType_kCableCar), DirectionsLeg_TransitType_kCableCar},
    {static_cast<int>(TripLeg_TransitType_kGondola), DirectionsLeg_TransitType_kGondola},
    {static_cast<int>(TripLeg_TransitType_kFunicular), DirectionsLeg_TransitType_kFunicular},
};

const std::unordered_map<int, DirectionsLeg_TravelMode> translate_travel_mode{
    {static_cast<int>(TripLeg_TravelMode_kDrive), DirectionsLeg_TravelMode_kDrive},
    {static_cast<int>(TripLeg_TravelMode_kPedestrian), DirectionsLeg_TravelMode_kPedestrian},
    {static_cast<int>(TripLeg_TravelMode_kBicycle), DirectionsLeg_TravelMode_kBicycle},
    {static_cast<int>(TripLeg_TravelMode_kTransit), DirectionsLeg_TravelMode_kTransit},
};

// Returns the trip directions based on the specified directions options
// and trip path. This method calls ManeuversBuilder::Build and
// NarrativeBuilder::Build to form the maneuver list. This method
// calls PopulateDirectionsLeg to transform the maneuver list into the
// trip directions.
void DirectionsBuilder::Build(Api& api) {
  const auto& options = api.options();
  for (auto& trip_route : *api.mutable_trip()->mutable_routes()) {
    auto& directions_route = *api.mutable_directions()->mutable_routes()->Add();
    for (auto& trip_path : *trip_route.mutable_legs()) {
      auto& trip_directions = *directions_route.mutable_legs()->Add();

      // Validate trip path node list
      if (trip_path.node_size() < 1) {
        throw valhalla_exception_t{210};
      }

      // Create an enhanced trip path from the specified trip_path
      EnhancedTripLeg etp(trip_path);

      // Produce maneuvers if desired
      std::list<Maneuver> maneuvers;
      if (options.directions_type() != DirectionsType::none) {
        // Update the heading of ~0 length edges
        UpdateHeading(&etp);

        ManeuversBuilder maneuversBuilder(options, &etp);
        maneuvers = maneuversBuilder.Build();

        // Create the instructions if desired
        if (options.directions_type() == DirectionsType::instructions) {
          std::unique_ptr<NarrativeBuilder> narrative_builder =
              NarrativeBuilderFactory::Create(options, &etp);
          narrative_builder->Build(maneuvers);
        }
      }

      // Return trip directions
      PopulateDirectionsLeg(options, &etp, maneuvers, trip_directions);
    }
  }
}

// Update the heading of ~0 length edges.
void DirectionsBuilder::UpdateHeading(EnhancedTripLeg* etp) {

  for (size_t x = 0; x < etp->node_size(); ++x) {
    auto prev_edge = etp->GetPrevEdge(x);
    auto curr_edge = etp->GetCurrEdge(x);
    auto next_edge = etp->GetNextEdge(x);

    // If very short edge and no headings
    if (curr_edge && (curr_edge->length_km() <= kMinEdgeLength) &&
        (curr_edge->begin_heading() == 0) && (curr_edge->end_heading() == 0)) {
      // Use next edge to set the current begin/end heading
      if (next_edge && (next_edge->length_km() > kMinEdgeLength)) {
        curr_edge->set_begin_heading(next_edge->begin_heading());
        curr_edge->set_end_heading(next_edge->begin_heading());
      }
      // Use prev edge to set the current begin/end heading
      else if (prev_edge && (prev_edge->length_km() > kMinEdgeLength)) {
        curr_edge->set_begin_heading(prev_edge->end_heading());
        curr_edge->set_end_heading(prev_edge->end_heading());
      }
    }
  }
}

// Returns the trip directions based on the specified directions options,
// trip path, and maneuver list.
void DirectionsBuilder::PopulateDirectionsLeg(const Options& options,
                                              EnhancedTripLeg* etp,
                                              std::list<Maneuver>& maneuvers,
                                              DirectionsLeg& trip_directions) {
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
      auto* maneuver_street_name = trip_maneuver->add_street_name();
      maneuver_street_name->set_value(street_name->value());
      maneuver_street_name->set_is_route_number(street_name->is_route_number());
    }

    // Set begin street names
    for (const auto& begin_street_name : maneuver.begin_street_names()) {
      auto* maneuver_begin_street_name = trip_maneuver->add_begin_street_name();
      maneuver_begin_street_name->set_value(begin_street_name->value());
      maneuver_begin_street_name->set_is_route_number(begin_street_name->is_route_number());
    }

    trip_maneuver->set_length(maneuver.length(options.units()));
    trip_maneuver->set_time(maneuver.time());
    trip_maneuver->set_begin_cardinal_direction(maneuver.begin_cardinal_direction());
    trip_maneuver->set_begin_heading(maneuver.begin_heading());
    trip_maneuver->set_turn_degree(maneuver.turn_degree());
    trip_maneuver->set_begin_shape_index(maneuver.begin_shape_index());
    trip_maneuver->set_end_shape_index(maneuver.end_shape_index());
    if (maneuver.portions_toll()) {
      trip_maneuver->set_portions_toll(maneuver.portions_toll());
    }

    trip_maneuver->set_has_time_restrictions(maneuver.has_time_restrictions());

    if (maneuver.portions_unpaved()) {
      trip_maneuver->set_portions_unpaved(maneuver.portions_unpaved());
    }

    if (maneuver.HasVerbalSuccinctTransitionInstruction()) {
      trip_maneuver->set_verbal_succinct_transition_instruction(
          maneuver.verbal_succinct_transition_instruction());
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
    if (maneuver.HasExitSign() || maneuver.HasGuideSign() || maneuver.HasJunctionNameSign()) {
      auto* trip_sign = trip_maneuver->mutable_sign();

      // Process exit number info
      if (maneuver.HasExitNumberSign()) {
        for (const auto& exit_number : maneuver.signs().exit_number_list()) {
          auto* trip_exit_number = trip_sign->mutable_exit_numbers()->Add();
          trip_exit_number->set_text(exit_number.text());
          trip_exit_number->set_is_route_number(exit_number.is_route_number());
          trip_exit_number->set_consecutive_count(exit_number.consecutive_count());
        }
      }

      // Process exit branch info
      if (maneuver.HasExitBranchSign()) {
        for (const auto& exit_branch : maneuver.signs().exit_branch_list()) {
          auto* trip_exit_onto_street = trip_sign->mutable_exit_onto_streets()->Add();
          trip_exit_onto_street->set_text(exit_branch.text());
          trip_exit_onto_street->set_is_route_number(exit_branch.is_route_number());
          trip_exit_onto_street->set_consecutive_count(exit_branch.consecutive_count());
        }
      }

      // Process exit toward info
      if (maneuver.HasExitTowardSign()) {
        for (const auto& exit_toward : maneuver.signs().exit_toward_list()) {
          auto* trip_exit_toward_location = trip_sign->mutable_exit_toward_locations()->Add();
          trip_exit_toward_location->set_text(exit_toward.text());
          trip_exit_toward_location->set_is_route_number(exit_toward.is_route_number());
          trip_exit_toward_location->set_consecutive_count(exit_toward.consecutive_count());
        }
      }

      // Process exit name info
      if (maneuver.HasExitNameSign()) {
        for (const auto& exit_name : maneuver.signs().exit_name_list()) {
          auto* trip_exit_name = trip_sign->mutable_exit_names()->Add();
          trip_exit_name->set_text(exit_name.text());
          trip_exit_name->set_is_route_number(exit_name.is_route_number());
          trip_exit_name->set_consecutive_count(exit_name.consecutive_count());
        }
      }

      // Process guide branch info
      if (maneuver.HasGuideBranchSign()) {
        for (const auto& guide_branch : maneuver.signs().guide_branch_list()) {
          auto* trip_guide_onto_street = trip_sign->mutable_guide_onto_streets()->Add();
          trip_guide_onto_street->set_text(guide_branch.text());
          trip_guide_onto_street->set_is_route_number(guide_branch.is_route_number());
          trip_guide_onto_street->set_consecutive_count(guide_branch.consecutive_count());
        }
      }

      // Process guide toward info
      if (maneuver.HasGuideTowardSign()) {
        for (const auto& guide_toward : maneuver.signs().guide_toward_list()) {
          auto* trip_guide_toward_location = trip_sign->mutable_guide_toward_locations()->Add();
          trip_guide_toward_location->set_text(guide_toward.text());
          trip_guide_toward_location->set_is_route_number(guide_toward.is_route_number());
          trip_guide_toward_location->set_consecutive_count(guide_toward.consecutive_count());
        }
      }

      // Process named junction info
      if (maneuver.HasJunctionNameSign()) {
        for (const auto& junction_name : maneuver.signs().junction_name_list()) {
          auto* trip_junction_name = trip_sign->mutable_junction_names()->Add();
          trip_junction_name->set_text(junction_name.text());
          trip_junction_name->set_is_route_number(junction_name.is_route_number());
          trip_junction_name->set_consecutive_count(junction_name.consecutive_count());
        }
      }
    }

    // Process the guidance views
    for (const auto& guidance_view : maneuver.guidance_views()) {
      trip_maneuver->add_guidance_views()->CopyFrom(guidance_view);
    }

    // Roundabout exit count
    if (maneuver.roundabout_exit_count() > 0) {
      trip_maneuver->set_roundabout_exit_count(maneuver.roundabout_exit_count());
    }

    // Set roundabout exit street names
    for (const auto& roundabout_exit_street_names : maneuver.roundabout_exit_street_names()) {
      auto* maneuver_roundabout_exit_street_names = trip_maneuver->add_roundabout_exit_street_names();
      maneuver_roundabout_exit_street_names->set_value(roundabout_exit_street_names->value());
      maneuver_roundabout_exit_street_names->set_is_route_number(
          roundabout_exit_street_names->is_route_number());
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
    if (maneuver.HasVerbalMultiCue()) {
      trip_maneuver->set_verbal_multi_cue(maneuver.HasVerbalMultiCue());
    }

    // To stay on
    if (maneuver.to_stay_on()) {
      trip_maneuver->set_to_stay_on(maneuver.to_stay_on());
    }

    // Travel mode
    trip_maneuver->set_travel_mode(translate_travel_mode.find(maneuver.travel_mode())->second);

    // Bss maneuver type
    trip_maneuver->set_bss_maneuver_type(maneuver.bss_maneuver_type());

    // Travel type
    switch (maneuver.travel_mode()) {
      case TripLeg_TravelMode_kDrive: {
        trip_maneuver->set_vehicle_type(translate_vehicle_type.find(maneuver.vehicle_type())->second);
        break;
      }
      case TripLeg_TravelMode_kPedestrian: {
        trip_maneuver->set_pedestrian_type(
            translate_pedestrian_type.find(maneuver.pedestrian_type())->second);
        break;
      }
      case TripLeg_TravelMode_kBicycle: {
        trip_maneuver->set_bicycle_type(translate_bicycle_type.find(maneuver.bicycle_type())->second);
        break;
      }
      case TripLeg_TravelMode_kTransit: {
        trip_maneuver->set_transit_type(translate_transit_type.find(maneuver.transit_type())->second);
        break;
      }
    }
  }

  // Populate summary
  trip_directions.mutable_summary()->set_length(etp->GetLength(options.units()));
  trip_directions.mutable_summary()->set_time(
      etp->node(etp->GetLastNodeIndex()).cost().elapsed_cost().seconds());
  auto mutable_bbox = trip_directions.mutable_summary()->mutable_bbox();
  mutable_bbox->mutable_min_ll()->set_lat(etp->bbox().min_ll().lat());
  mutable_bbox->mutable_min_ll()->set_lng(etp->bbox().min_ll().lng());
  mutable_bbox->mutable_max_ll()->set_lat(etp->bbox().max_ll().lat());
  mutable_bbox->mutable_max_ll()->set_lng(etp->bbox().max_ll().lng());

  // Populate shape
  trip_directions.set_shape(etp->shape());

  // Populate has_time_restrictions
  bool has_time_restrictions = false;
  for (const auto& node : etp->node()) {
    has_time_restrictions = node.edge().has_time_restrictions() || has_time_restrictions;
  }
  trip_directions.mutable_summary()->set_has_time_restrictions(has_time_restrictions);
}

} // namespace odin
} // namespace valhalla
