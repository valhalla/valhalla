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

TripDirections DirectionsBuilder::Build(const DirectionsOptions& directions_options,
                                        TripPath& trip_path) {
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
  return PopulateTripDirections(directions_options, trip_path, maneuvers);
}

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

TripDirections DirectionsBuilder::PopulateTripDirections(
    const DirectionsOptions& directions_options, TripPath& trip_path,
    std::list<Maneuver>& maneuvers) {
  TripDirections trip_directions;

  // Populate trip and leg IDs
  trip_directions.set_trip_id(trip_path.trip_id());
  trip_directions.set_leg_id(trip_path.leg_id());
  trip_directions.set_leg_count(trip_path.leg_count());

  // Populate locations
  for (const auto& path_location : trip_path.location()) {
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
  }

  // Populate maneuvers
  float leg_length = 0.0f;
  uint32_t leg_time = 0;
  for (const auto& maneuver : maneuvers) {
    auto* trip_maneuver = trip_directions.add_maneuver();
    trip_maneuver->set_type(maneuver.type());
    trip_maneuver->set_text_instruction(maneuver.instruction());
    for (const auto& street_name : maneuver.street_names()) {
      trip_maneuver->add_street_name(street_name->value());
    }
    trip_maneuver->set_length(maneuver.distance());
    leg_length += maneuver.distance();
    trip_maneuver->set_time(maneuver.time());
    leg_time += maneuver.time();
    trip_maneuver->set_begin_cardinal_direction(
        maneuver.begin_cardinal_direction());
    trip_maneuver->set_begin_heading(maneuver.begin_heading());
    trip_maneuver->set_begin_shape_index(maneuver.begin_shape_index());
    trip_maneuver->set_end_shape_index(maneuver.end_shape_index());
    trip_maneuver->set_portions_toll(maneuver.portions_toll());
    trip_maneuver->set_portions_unpaved(maneuver.portions_unpaved());
  }

  // Populate summary
  trip_directions.mutable_summary()->set_length(leg_length);
  trip_directions.mutable_summary()->set_time(leg_time);

  // Populate shape
  trip_directions.set_shape(trip_path.shape());

  return trip_directions;
}

}
}
