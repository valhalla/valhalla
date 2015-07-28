#include <iostream>

#include "odin/enhancedtrippath.h"
#include "odin/directionsbuilder.h"
#include "odin/maneuversbuilder.h"
#include "odin/narrativebuilder.h"

namespace {
// Minimum edge length
constexpr auto kMinEdgeLength = 0.003f;
}

namespace valhalla {
namespace odin {

DirectionsBuilder::DirectionsBuilder() {
}

namespace {
std::string GetStreetName(std::vector<std::string>& maneuver_names) {
  std::string street_name;
  if (maneuver_names.empty()) {
    street_name = "unnamed road";
  } else {
    for (const auto& name : maneuver_names) {
      if (!street_name.empty()) {
        street_name += "/";
      }
      street_name += name;
    }
  }
  return street_name;
}

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
    throw std::runtime_error("Trip path does not have any nodes");
  }

  EnhancedTripPath* etp = static_cast<EnhancedTripPath*>(&trip_path);

  // Update the heading of ~0 length edges
  UpdateHeading(etp);

  // Create maneuvers
  ManeuversBuilder maneuversBuilder(directions_options, etp);
  auto maneuvers = maneuversBuilder.Build();

  // Create the narrative
  // TODO - factory
  NarrativeBuilder::Build(directions_options, maneuvers);

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
      }
      else if (next_edge && (next_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_begin_heading(next_edge->begin_heading());
      }

      // Set the current end heading
      if (next_edge && (next_edge->length() >= kMinEdgeLength)) {
        curr_edge->set_end_heading(next_edge->begin_heading());
      }
      else if (prev_edge && (prev_edge->length() >= kMinEdgeLength)) {
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
  for (const auto& path_location : etp->location()) {
    auto* direction_location = trip_directions.add_location();
    direction_location->mutable_ll()->set_lat(path_location.ll().lat());
    direction_location->mutable_ll()->set_lng(path_location.ll().lng());
    if (path_location.type() == TripPath_Location_Type_kThrough) {
      direction_location->set_type(TripDirections_Location_Type_kThrough);
    } else {
      direction_location->set_type(TripDirections_Location_Type_kBreak);
    }

    if (path_location.has_heading())
      direction_location->set_heading(path_location.heading());
    if (path_location.has_name())
      direction_location->set_name(path_location.name());
    if (path_location.has_street())
      direction_location->set_street(path_location.street());
    if (path_location.has_city())
      direction_location->set_city(path_location.city());
    if (path_location.has_state())
      direction_location->set_state(path_location.state());
    if (path_location.has_postal_code())
      direction_location->set_postal_code(path_location.postal_code());
    if (path_location.has_country())
      direction_location->set_country(path_location.country());
    if (path_location.has_date_time())
      direction_location->set_date_time(path_location.date_time());
  }

  // Populate maneuvers
  float leg_length = 0.0f;
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
    leg_length += maneuver.length(directions_options.units());
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

  }

  // Populate summary
  trip_directions.mutable_summary()->set_length(leg_length);
  trip_directions.mutable_summary()->set_time(
      etp->node(etp->GetLastNodeIndex()).elapsed_time());

  // Populate shape
  trip_directions.set_shape(etp->shape());

  return trip_directions;
}

}
}
