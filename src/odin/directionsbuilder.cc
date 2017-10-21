#include <iostream>
#include <unordered_map>

#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"
#include "odin/enhancedtrippath.h"
#include "odin/directionsbuilder.h"
#include "odin/maneuversbuilder.h"
#include "odin/narrativebuilder.h"
#include "odin/narrative_builder_factory.h"
#include "exception.h"

namespace {
// Minimum edge length (~10 feet)
constexpr auto kMinEdgeLength = 0.003f;

}

namespace valhalla {
namespace odin {

const std::unordered_map<int, TripDirections_VehicleType> translate_vehicle_type {
  { static_cast<int>(TripPath_VehicleType_kCar), TripDirections_VehicleType_kCar },
  { static_cast<int>(TripPath_VehicleType_kMotorcycle), TripDirections_VehicleType_kMotorcycle },
  { static_cast<int>(TripPath_VehicleType_kAutoBus), TripDirections_VehicleType_kAutoBus },
  { static_cast<int>(TripPath_VehicleType_kTractorTrailer), TripDirections_VehicleType_kTractorTrailer },
  { static_cast<int>(TripPath_VehicleType_kMotorScooter), TripDirections_VehicleType_kMotorScooter },
};

const std::unordered_map<int, TripDirections_PedestrianType> translate_pedestrian_type {
  { static_cast<int>(TripPath_PedestrianType_kFoot), TripDirections_PedestrianType_kFoot },
  { static_cast<int>(TripPath_PedestrianType_kWheelchair), TripDirections_PedestrianType_kWheelchair },
  { static_cast<int>(TripPath_PedestrianType_kSegway), TripDirections_PedestrianType_kSegway },
};

const std::unordered_map<int, TripDirections_BicycleType> translate_bicycle_type {
  { static_cast<int>(TripPath_BicycleType_kRoad), TripDirections_BicycleType_kRoad },
  { static_cast<int>(TripPath_BicycleType_kCross), TripDirections_BicycleType_kCross },
  { static_cast<int>(TripPath_BicycleType_kHybrid), TripDirections_BicycleType_kHybrid },
  { static_cast<int>(TripPath_BicycleType_kMountain), TripDirections_BicycleType_kMountain },
};

const std::unordered_map<int, TripDirections_TransitType> translate_transit_type {
  { static_cast<int>(TripPath_TransitType_kTram), TripDirections_TransitType_kTram },
  { static_cast<int>(TripPath_TransitType_kMetro), TripDirections_TransitType_kMetro },
  { static_cast<int>(TripPath_TransitType_kRail), TripDirections_TransitType_kRail },
  { static_cast<int>(TripPath_TransitType_kBus), TripDirections_TransitType_kBus },
  { static_cast<int>(TripPath_TransitType_kFerry), TripDirections_TransitType_kFerry },
  { static_cast<int>(TripPath_TransitType_kCableCar), TripDirections_TransitType_kCableCar },
  { static_cast<int>(TripPath_TransitType_kGondola), TripDirections_TransitType_kGondola },
  { static_cast<int>(TripPath_TransitType_kFunicular), TripDirections_TransitType_kFunicular },
};

const std::unordered_map<int, TripDirections_TravelMode> translate_travel_mode {
  { static_cast<int>(TripPath_TravelMode_kDrive), TripDirections_TravelMode_kDrive },
  { static_cast<int>(TripPath_TravelMode_kPedestrian), TripDirections_TravelMode_kPedestrian },
  { static_cast<int>(TripPath_TravelMode_kBicycle), TripDirections_TravelMode_kBicycle },
  { static_cast<int>(TripPath_TravelMode_kTransit), TripDirections_TravelMode_kTransit },
};

DirectionsBuilder::DirectionsBuilder() {
}

// Returns the trip directions based on the specified directions options
// and trip path. This method calls ManeuversBuilder::Build and
// NarrativeBuilder::Build to form the maneuver list. This method
// calls PopulateTripDirections to transform the maneuver list into the
// trip directions.
TripDirections DirectionsBuilder::Build(
    const DirectionsOptions& directions_options, TripPath& trip_path) {
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
TripDirections DirectionsBuilder::PopulateTripDirections(
    const DirectionsOptions& directions_options, EnhancedTripPath* etp,
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
    trip_maneuver->set_begin_cardinal_direction(
        maneuver.begin_cardinal_direction());
    trip_maneuver->set_begin_heading(maneuver.begin_heading());
    trip_maneuver->set_begin_shape_index(maneuver.begin_shape_index());
    trip_maneuver->set_end_shape_index(maneuver.end_shape_index());
    if (maneuver.portions_toll())
      trip_maneuver->set_portions_toll(maneuver.portions_toll());
    if (maneuver.portions_unpaved())
      trip_maneuver->set_portions_unpaved(maneuver.portions_unpaved());

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
        auto* trip_exit_number_elements = trip_sign
            ->mutable_exit_number_elements();
        for (const auto& exit_number : maneuver.signs().exit_number_list()) {
          auto* trip_exit_number_element = trip_exit_number_elements->Add();
          trip_exit_number_element->set_text(exit_number.text());
          trip_exit_number_element->set_consecutive_count(
              exit_number.consecutive_count());
        }
      }

      // Process exit branch info
      if (maneuver.HasExitBranchSign()) {
        auto* trip_exit_branch_elements = trip_sign
            ->mutable_exit_branch_elements();
        for (const auto& exit_branch : maneuver.signs().exit_branch_list()) {
          auto* trip_exit_branch_element = trip_exit_branch_elements->Add();
          trip_exit_branch_element->set_text(exit_branch.text());
          trip_exit_branch_element->set_consecutive_count(
              exit_branch.consecutive_count());
        }
      }

      // Process exit toward info
      if (maneuver.HasExitTowardSign()) {
        auto* trip_exit_toward_elements = trip_sign
            ->mutable_exit_toward_elements();
        for (const auto& exit_toward : maneuver.signs().exit_toward_list()) {
          auto* trip_exit_toward_element = trip_exit_toward_elements->Add();
          trip_exit_toward_element->set_text(exit_toward.text());
          trip_exit_toward_element->set_consecutive_count(
              exit_toward.consecutive_count());
        }
      }

      // Process exit name info
      if (maneuver.HasExitNameSign()) {
        auto* trip_exit_name_elements = trip_sign->mutable_exit_name_elements();
        for (const auto& exit_name : maneuver.signs().exit_name_list()) {
          auto* trip_exit_name_element = trip_exit_name_elements->Add();
          trip_exit_name_element->set_text(exit_name.text());
          trip_exit_name_element->set_consecutive_count(
              exit_name.consecutive_count());
        }
      }

    }

    // Roundabout exit count
    if (maneuver.roundabout_exit_count() > 0) {
      trip_maneuver->set_roundabout_exit_count(
          maneuver.roundabout_exit_count());
    }

    // Depart instructions
    if (!maneuver.depart_instruction().empty()) {
      trip_maneuver->set_depart_instruction(maneuver.depart_instruction());
    }
    if (!maneuver.verbal_depart_instruction().empty()) {
      trip_maneuver->set_verbal_depart_instruction(
          maneuver.verbal_depart_instruction());
    }

    // Arrive instructions
    if (!maneuver.arrive_instruction().empty()) {
      trip_maneuver->set_arrive_instruction(maneuver.arrive_instruction());
    }
    if (!maneuver.verbal_arrive_instruction().empty()) {
      trip_maneuver->set_verbal_arrive_instruction(
          maneuver.verbal_arrive_instruction());
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
        trip_transit_info->set_operator_onestop_id(
            transit_route.operator_onestop_id);
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
    if (maneuver.verbal_multi_cue())
      trip_maneuver->set_verbal_multi_cue(maneuver.verbal_multi_cue());

    // Travel mode
    trip_maneuver->set_travel_mode(translate_travel_mode.find(maneuver.travel_mode())->second);

    // Travel type
    switch (maneuver.travel_mode()) {
      case TripPath_TravelMode_kDrive: {
        trip_maneuver->set_vehicle_type(
            translate_vehicle_type.find(maneuver.vehicle_type())->second);
        break;
      }
      case TripPath_TravelMode_kPedestrian: {
        trip_maneuver->set_pedestrian_type(
            translate_pedestrian_type.find(maneuver.pedestrian_type())->second);
        break;
      }
      case TripPath_TravelMode_kBicycle: {
        trip_maneuver->set_bicycle_type(
            translate_bicycle_type.find(maneuver.bicycle_type())->second);
        break;
      }
      case TripPath_TravelMode_kTransit: {
        trip_maneuver->set_transit_type(
            translate_transit_type.find(maneuver.transit_type())->second);
        break;
      }
    }
  }

  // Populate summary
  trip_directions.mutable_summary()->set_length(
      etp->GetLength(directions_options.units()));
  trip_directions.mutable_summary()->set_time(
      etp->node(etp->GetLastNodeIndex()).elapsed_time());
  auto mutable_bbox = trip_directions.mutable_summary()->mutable_bbox();
  mutable_bbox->mutable_min_ll()->set_lat(etp->bbox().min_ll().lat());
  mutable_bbox->mutable_min_ll()->set_lng(etp->bbox().min_ll().lng());
  mutable_bbox->mutable_max_ll()->set_lat(etp->bbox().max_ll().lat());
  mutable_bbox->mutable_max_ll()->set_lng(etp->bbox().max_ll().lng());

  // Populate shape
  trip_directions.set_shape(etp->shape());

  return trip_directions;
}

}
}
