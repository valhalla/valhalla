#include <iostream>

#include "odin/enhancedtrippath.h"
#include "odin/directionsbuilder.h"
#include "odin/maneuversbuilder.h"
#include "odin/narrativebuilder.h"

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

TripDirections DirectionsBuilder::Build(TripPath& trip_path) {
  EnhancedTripPath* etp = static_cast<EnhancedTripPath*>(&trip_path);

  // Create maneuvers
  ManeuversBuilder maneuversBuilder(etp);
  auto maneuvers = maneuversBuilder.Build();

  // Create the narrative
  // TODO - factory
  NarrativeBuilder::Build(maneuvers);

  // Return trip directions
  return PopulateTripDirections(trip_path, maneuvers);
}

TripDirections DirectionsBuilder::PopulateTripDirections(
    TripPath& trip_path, std::list<Maneuver>& maneuvers) {
  TripDirections trip_directions;

  // Populate trip and leg IDs
  trip_directions.set_trip_id(trip_path.trip_id());
  trip_directions.set_leg_id(trip_path.leg_id());

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
    direction_location->set_heading(path_location.heading());
    direction_location->set_name(path_location.name());
    direction_location->set_street(path_location.street());
    direction_location->set_city(path_location.city());
    direction_location->set_state(path_location.state());
    direction_location->set_postal_code(path_location.postal_code());
    direction_location->set_country(path_location.country());
  }

  // Populate maneuvers
  for (const auto& maneuver : maneuvers) {
    auto* trip_maneuver = trip_directions.add_maneuver();
    trip_maneuver->set_type(maneuver.type());
    trip_maneuver->set_text_instruction(maneuver.instruction());
    for (const auto& street_name : maneuver.street_names()) {
      trip_maneuver->add_street_name(street_name->value());
    }
    trip_maneuver->set_length(maneuver.distance());
    trip_maneuver->set_time(maneuver.time());
    trip_maneuver->set_begin_cardinal_direction(
        maneuver.begin_cardinal_direction());
    trip_maneuver->set_begin_heading(maneuver.begin_heading());
    trip_maneuver->set_begin_shape_index(maneuver.begin_shape_index());
    trip_maneuver->set_end_shape_index(maneuver.end_shape_index());
    trip_maneuver->set_portions_toll(maneuver.portions_toll());
    trip_maneuver->set_portions_unpaved(maneuver.portions_unpaved());
  }

  // Populate shape
  trip_directions.set_shape(trip_path.shape());

  return trip_directions;
}

}
}
