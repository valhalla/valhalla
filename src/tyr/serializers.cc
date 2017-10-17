#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/pointll.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "exception.h"
#include "odin/util.h"
#include "proto/directions_options.pb.h"
#include "tyr/serializers.h"


using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {

  namespace osrm_serializers {
    /*
    OSRM output looks like this:
    {
        "hint_data": {
            "locations": [
                "_____38_SADaFQQAKwEAABEAAAAAAAAAdgAAAFfLwga4tW0C4P6W-wAARAA",
                "fzhIAP____8wFAQA1AAAAC8BAAAAAAAAAAAAAP____9Uu20CGAiX-wAAAAA"
            ],
            "checksum": 2875622111
        },
        "route_name": [ "West 26th Street", "Madison Avenue" ],
        "via_indices": [ 0, 9 ],
        "found_alternative": false,
        "route_summary": {
            "end_point": "West 29th Street",
            "start_point": "West 26th Street",
            "total_time": 145,
            "total_distance": 878
        },
        "via_points": [ [ 40.744377, -73.990433 ], [40.745811, -73.988075 ] ],
        "route_instructions": [
            [ "10", "West 26th Street", 216, 0, 52, "215m", "SE", 118 ],
            [ "1", "East 26th Street", 153, 2, 29, "153m", "SE", 120 ],
            [ "7", "Madison Avenue", 237, 3, 25, "236m", "NE", 29 ],
            [ "7", "East 29th Street", 155, 6, 29, "154m", "NW", 299 ],
            [ "1", "West 29th Street", 118, 7, 21, "117m", "NW", 299 ],
            [ "15", "", 0, 8, 0, "0m", "N", 0 ]
        ],
        "route_geometry": "ozyulA~p_clCfc@ywApTar@li@ybBqe@c[ue@e[ue@i[ci@dcB}^rkA",
        "status_message": "Found route between points",
        "status": 0
    }
    */

    json::ArrayPtr route_name(const std::list<valhalla::odin::TripDirections>& legs){
      auto route_name = json::array({});
      //first one
      if(legs.front().maneuver(0).street_name_size() > 0)
        route_name->push_back(legs.front().maneuver(0).street_name(0));
      //the rest
      for(const auto& leg : legs) {
        if(leg.maneuver(leg.maneuver_size() - 1).street_name_size() > 0)
          route_name->push_back(leg.maneuver(leg.maneuver_size() - 1).street_name(0));
      }
      return route_name;
    }

    json::ArrayPtr via_indices(const std::list<valhalla::odin::TripDirections>& legs){
      //first one
      auto via_indices = json::array({static_cast<uint64_t>(0)});
      //the rest
      for(const auto& leg : legs)
        via_indices->push_back(static_cast<uint64_t>(leg.maneuver_size() - 1) + boost::get<uint64_t>(via_indices->back()));
      return via_indices;
    }

    json::MapPtr route_summary(const std::list<valhalla::odin::TripDirections>& legs){
      auto route_summary = json::map({});

      if(legs.front().maneuver(0).street_name_size() > 0)
        route_summary->emplace("start_point", legs.front().maneuver(0).street_name(0));
      else
        route_summary->emplace("start_point", string(""));

      if(legs.back().maneuver(legs.back().maneuver_size() - 1).street_name_size() > 0)
        route_summary->emplace("end_point", legs.back().maneuver(legs.back().maneuver_size() - 1).street_name(0));
      else
        route_summary->emplace("end_point", string(""));

      uint32_t seconds = 0;
      float kilometers = 0.f;
      for(const auto& leg : legs) {
        kilometers += leg.summary().length();
        seconds += leg.summary().time();
      }

      route_summary->emplace("total_time", static_cast<uint64_t>(seconds));
      route_summary->emplace("total_distance", static_cast<uint64_t>((kilometers * 1000.f) + .5f));
      return route_summary;
    }

    json::ArrayPtr via_points(const std::list<valhalla::odin::TripDirections>& legs){
      //first one
      auto via_points = json::array({
        json::array({json::fp_t{legs.front().location(0).ll().lat(),6}, json::fp_t{legs.front().location(0).ll().lng(),6}})
      });
      //the rest
      for(const auto& leg : legs) {
        for(int i = 1; i < leg.location_size(); ++i) {
          const auto& location = leg.location(i);
          via_points->emplace_back(json::array({json::fp_t{location.ll().lat(),6}, json::fp_t{location.ll().lng(),6}}));
        }
      }
      return via_points;
    }

    const std::unordered_map<int, std::string> maneuver_type = {
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kNone),             "0" },//NoTurn = 0,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kContinue),         "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kBecomes),          "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampStraight),     "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayStraight),     "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kMerge),            "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kFerryEnter),       "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kFerryExit),        "1" },//GoStraight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSlightRight),      "2" },//TurnSlightRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRight),            "3" },//TurnRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampRight),        "3" },//TurnRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kExitRight),        "3" },//TurnRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayRight),        "3" },//TurnRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSharpRight),       "4" },//TurnSharpRight,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kUturnLeft),        "5" },//UTurn,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kUturnRight),       "5" },//UTurn,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSharpLeft),        "6" },//TurnSharpLeft,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kLeft),             "7" },//TurnLeft,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampLeft),         "7" },//TurnLeft,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kExitLeft),         "7" },//TurnLeft,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayLeft),         "7" },//TurnLeft,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSlightLeft),       "8" },//TurnSlightLeft,
        //{ static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_k),               "9" },//ReachViaLocation,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutEnter),  "11" },//EnterRoundAbout,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutExit),   "12" },//LeaveRoundAbout,
        //{ static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_k),               "13" },//StayOnRoundAbout,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStart),            "14" },//StartAtEndOfStreet,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStartRight),       "14" },//StartAtEndOfStreet,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStartLeft),        "14" },//StartAtEndOfStreet,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestination),      "15" },//ReachedYourDestination,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestinationRight), "15" },//ReachedYourDestination,
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestinationLeft),  "15" },//ReachedYourDestination,
        //{ static_cast<int>valhalla::odin::TripDirections_Maneuver_Type_k),                "16" },//EnterAgainstAllowedDirection,
        //{ static_cast<int>valhalla::odin::TripDirections_Maneuver_Type_k),                "17" },//LeaveAgainstAllowedDirection
    };

    const std::unordered_map<int, std::string> cardinal_direction_string = {
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorth),     "N" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorthEast), "NE" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kEast),      "E" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouthEast), "SE" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouth),     "S" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouthWest), "SW" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kWest),      "W" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorthWest), "NW" }
    };

    json::ArrayPtr route_instructions(const std::list<valhalla::odin::TripDirections>& legs){
      auto route_instructions = json::array({});
      for(const auto& leg : legs) {
        for(const auto& maneuver : leg.maneuver()) {
          //if we dont know the type of maneuver then skip it
          auto maneuver_text = maneuver_type.find(static_cast<int>(maneuver.type()));
          if(maneuver_text == maneuver_type.end())
            continue;

          //length
          std::ostringstream length;
          length << static_cast<uint64_t>(maneuver.length()*1000.f) << "m";

          //json
          route_instructions->emplace_back(json::array({
            maneuver_text->second, //maneuver type
            (maneuver.street_name_size() ? maneuver.street_name(0) : string("")), //street name
            static_cast<uint64_t>(maneuver.length() * 1000.f), //length in meters
            static_cast<uint64_t>(maneuver.begin_shape_index()), //index in the shape
            static_cast<uint64_t>(maneuver.time()), //time in seconds
            length.str(), //length as a string with a unit suffix
            cardinal_direction_string.find(static_cast<int>(maneuver.begin_cardinal_direction()))->second, // one of: N S E W NW NE SW SE
            static_cast<uint64_t>(maneuver.begin_heading())
          }));
        }
      }
      return route_instructions;
    }

    std::string shape(const std::list<valhalla::odin::TripDirections>& legs) {
      if(legs.size() == 1)
        return legs.front().shape();

      //TODO: there is a tricky way to do this... since the end of each leg is the same as the beginning
      //we essentially could just peel off the first encoded shape point of all the legs (but the first)
      //this way we wouldn't really have to do any decoding (would be far faster). it might even be the case
      //that the string length of the first number is a fixed length (which would be great!) have to have a look
      //should make this a function in midgard probably so the logic is all in the same place
      std::vector<std::pair<float, float> > decoded;
      for(const auto& leg : legs) {
        auto decoded_leg = midgard::decode<std::vector<std::pair<float, float> > >(leg.shape());
        decoded.insert(decoded.end(), decoded.size() ? decoded_leg.begin() + 1 : decoded_leg.begin(), decoded_leg.end());
      }
      return midgard::encode(decoded);
    }

    json::MapPtr serialize(const valhalla::odin::DirectionsOptions& directions_options,
      const std::list<valhalla::odin::TripDirections>& legs) {
      auto json = json::map
      ({
        {"hint_data", json::map
          ({
            {"locations", json::array({ string(""), string("") })}, //TODO: are these internal ids?
            {"checksum", static_cast<uint64_t>(0)} //TODO: what is this exactly?
          })
        },
        {"route_name", route_name(legs)}, //TODO: list of all of the streets or just the via points?
        {"via_indices", via_indices(legs)}, //maneuver index
        {"found_alternative", static_cast<bool>(false)}, //no alt route support
        {"route_summary", route_summary(legs)}, //start/end name, total time/distance
        {"via_points", via_points(legs)}, //array of lat,lng pairs
        {"route_instructions", route_instructions(legs)}, //array of maneuvers
        {"route_geometry", shape(legs)}, //polyline encoded shape
        {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
        {"status", static_cast<uint64_t>(0)} //0 success or 207 no route
      });

      return json;
    }
  }

  namespace valhalla_serializers {
    /*
    valhalla output looks like this:
    {
        "trip":
    {
        "status": 0,
        "locations": [
           {
            "longitude": -76.4791,
            "latitude": 40.4136,
             "stopType": 0
           },
           {
            "longitude": -76.5352,
            "latitude": 40.4029,
            "stopType": 0
           }
         ],
        "units": "kilometers"
        "summary":
    {
        "distance": 4973,
        "time": 325
    },
    "legs":
    [
      {
          "summary":
      {
          "distance": 4973,
          "time": 325
      },
      "maneuvers":
      [
        {
            "beginShapeIndex": 0,
            "distance": 633,
            "writtenInstruction": "Start out going west on West Market Street.",
            "streetNames":
            [
                "West Market Street"
            ],
            "type": 1,
            "time": 41
        },
        {
            "beginShapeIndex": 7,
            "distance": 4340,
            "writtenInstruction": "Continue onto Jonestown Road.",
            "streetNames":
            [
                "Jonestown Road"
            ],
            "type": 8,
            "time": 284
        },
        {
            "beginShapeIndex": 40,
            "distance": 0,
            "writtenInstruction": "You have arrived at your destination.",
            "type": 4,
            "time": 0
        }
    ],
    "shape": "gysalAlg|zpC~Clt@tDtx@hHfaBdKl{BrKbnApGro@tJrz@jBbQj@zVt@lTjFnnCrBz}BmFnoB]pHwCvm@eJxtATvXTnfAk@|^z@rGxGre@nTpnBhBbQvXduCrUr`Edd@naEja@~gAhk@nzBxf@byAfm@tuCvDtOvNzi@|jCvkKngAl`HlI|}@`N`{Adx@pjE??xB|J"
    }
    ],
    "status_message": "Found route between points"
    },
    "id": "work route"
    }
    */
    using namespace std;

    json::MapPtr summary(const std::list<valhalla::odin::TripDirections>& legs){

      uint64_t time = 0;
      long double length = 0;
      AABB2<PointLL> bbox(10000.0f, 10000.0f, -10000.0f, -10000.0f);
      for(const auto& leg : legs) {
        time += static_cast<uint64_t>(leg.summary().time());
        length += leg.summary().length();

        AABB2<PointLL> leg_bbox(leg.summary().bbox().min_ll().lng(),
                                leg.summary().bbox().min_ll().lat(),
                                leg.summary().bbox().max_ll().lng(),
                                leg.summary().bbox().max_ll().lat());
        bbox.Expand(leg_bbox);
      }

      auto route_summary = json::map({});
      route_summary->emplace("time", time);
      route_summary->emplace("length", json::fp_t{length, 3});
      route_summary->emplace("min_lat", json::fp_t{bbox.miny(), 6});
      route_summary->emplace("min_lon", json::fp_t{bbox.minx(), 6});
      route_summary->emplace("max_lat", json::fp_t{bbox.maxy(), 6});
      route_summary->emplace("max_lon", json::fp_t{bbox.maxx(), 6});
      LOG_DEBUG("trip_time::" + std::to_string(time) +"s");
      return route_summary;
    }

    json::ArrayPtr locations(const std::list<valhalla::odin::TripDirections>& legs){
      auto locations = json::array({});

      int index = 0;
      for(auto leg = legs.begin(); leg != legs.end(); ++leg) {
        for(auto location = leg->location().begin() + index; location != leg->location().end(); ++location) {
          index = 1;
          auto loc = json::map({});
          if (location->type() == odin::Location_Type_kThrough) {
            loc->emplace("type", std::string("through"));
          } else {
            loc->emplace("type", std::string("break"));
          }
          loc->emplace("lat", json::fp_t{location->ll().lat(), 6});
          loc->emplace("lon",json::fp_t{location->ll().lng(), 6});
          if (!location->name().empty())
            loc->emplace("name",location->name());
          if (!location->street().empty())
            loc->emplace("street",location->street());
          if (!location->city().empty())
            loc->emplace("city",location->city());
          if (!location->state().empty())
            loc->emplace("state",location->state());
          if (!location->postal_code().empty())
            loc->emplace("postal_code",location->postal_code());
          if (!location->country().empty())
            loc->emplace("country",location->country());
          if (location->has_heading())
            loc->emplace("heading",static_cast<uint64_t>(location->heading()));
          if (!location->date_time().empty())
            loc->emplace("date_time",location->date_time());
          if (location->has_side_of_street()) {
            if (location->side_of_street() == odin::Location_SideOfStreet_kLeft)
              loc->emplace("side_of_street", std::string("left"));
            else if (location->side_of_street() == odin::Location_SideOfStreet_kRight)
              loc->emplace("side_of_street", std::string("right"));
          }
          if (location->has_original_index())
            loc->emplace("original_index",static_cast<uint64_t>(location->original_index()));

          //loc->emplace("sideOfStreet",location->side_of_street());

          locations->emplace_back(loc);
        }
      }

      return locations;
    }

    const std::unordered_map<int, std::string> vehicle_to_string {
      { static_cast<int>(TripDirections_VehicleType_kCar), "car" },
      { static_cast<int>(TripDirections_VehicleType_kMotorcycle), "motorcycle" },
      { static_cast<int>(TripDirections_VehicleType_kAutoBus), "bus" },
      { static_cast<int>(TripDirections_VehicleType_kTractorTrailer), "tractor_trailer" },
      { static_cast<int>(TripDirections_VehicleType_kMotorScooter), "motor_scooter" },
    };

    std::unordered_map<int, std::string> pedestrian_to_string {
      { static_cast<int>(TripDirections_PedestrianType_kFoot), "foot" },
      { static_cast<int>(TripDirections_PedestrianType_kWheelchair), "wheelchair" },
      { static_cast<int>(TripDirections_PedestrianType_kSegway), "segway" },
    };

    std::unordered_map<int, std::string> bicycle_to_string {
      { static_cast<int>(TripDirections_BicycleType_kRoad), "road" },
      { static_cast<int>(TripDirections_BicycleType_kCross), "cross" },
      { static_cast<int>(TripDirections_BicycleType_kHybrid), "hybrid" },
      { static_cast<int>(TripDirections_BicycleType_kMountain), "mountain" },
    };

    std::unordered_map<int, std::string> transit_to_string {
      { static_cast<int>(TripDirections_TransitType_kTram), "tram" },
      { static_cast<int>(TripDirections_TransitType_kMetro), "metro" },
      { static_cast<int>(TripDirections_TransitType_kRail), "rail" },
      { static_cast<int>(TripDirections_TransitType_kBus), "bus" },
      { static_cast<int>(TripDirections_TransitType_kFerry), "ferry" },
      { static_cast<int>(TripDirections_TransitType_kCableCar), "cable_car" },
      { static_cast<int>(TripDirections_TransitType_kGondola), "gondola" },
      { static_cast<int>(TripDirections_TransitType_kFunicular), "funicular" },
    };

    std::pair<std::string, std::string> travel_mode_type(const valhalla::odin::TripDirections_Maneuver& maneuver) {
      switch (maneuver.travel_mode()) {
        case TripDirections_TravelMode_kDrive: {
          auto i = maneuver.has_vehicle_type() ? vehicle_to_string.find(maneuver.vehicle_type()) : vehicle_to_string.cend();
          return i == vehicle_to_string.cend() ? make_pair("drive", "car") : make_pair("drive", i->second);
        }
        case TripDirections_TravelMode_kPedestrian: {
          auto i = maneuver.has_pedestrian_type() ? pedestrian_to_string.find(maneuver.pedestrian_type()) : pedestrian_to_string.cend();
          return i == pedestrian_to_string.cend() ? make_pair("pedestrian", "foot") : make_pair("pedestrian", i->second);
        }
        case TripDirections_TravelMode_kBicycle: {
          auto i = maneuver.has_bicycle_type() ? bicycle_to_string.find(maneuver.bicycle_type()) : bicycle_to_string.cend();
          return i == bicycle_to_string.cend() ? make_pair("bicycle", "road") : make_pair("bicycle", i->second);
        }
        case TripDirections_TravelMode_kTransit: {
          auto i = maneuver.has_transit_type() ? transit_to_string.find(maneuver.transit_type()) : transit_to_string.cend();
          return i == transit_to_string.cend() ? make_pair("transit", "rail") : make_pair("transit", i->second);
        }
      }
    }

    json::ArrayPtr legs(const std::list<valhalla::odin::TripDirections>& directions_legs){

      // TODO: multiple legs.
      auto legs = json::array({});
      for(const auto& directions_leg : directions_legs) {
        auto leg = json::map({});
        auto summary = json::map({});
        auto maneuvers = json::array({});

        for(const auto& maneuver : directions_leg.maneuver()) {

          auto man = json::map({});

          // Maneuver type
          man->emplace("type", static_cast<uint64_t>(maneuver.type()));

          // Instruction and verbal instructions
          man->emplace("instruction", maneuver.text_instruction());
          if (maneuver.has_verbal_transition_alert_instruction()) {
            man->emplace("verbal_transition_alert_instruction",
                         maneuver.verbal_transition_alert_instruction());
          }
          if (maneuver.has_verbal_pre_transition_instruction()) {
            man->emplace("verbal_pre_transition_instruction",
                         maneuver.verbal_pre_transition_instruction());
          }
          if (maneuver.has_verbal_post_transition_instruction()) {
            man->emplace("verbal_post_transition_instruction",
                         maneuver.verbal_post_transition_instruction());
          }

          // Set street names
          if (maneuver.street_name_size() > 0) {
            auto street_names = json::array({});
            for (int i = 0; i < maneuver.street_name_size(); i++)
              street_names->emplace_back(maneuver.street_name(i));
            man->emplace("street_names", std::move(street_names));
          }

          // Set begin street names
          if (maneuver.begin_street_name_size() > 0) {
            auto begin_street_names = json::array({});
            for (int i = 0; i < maneuver.begin_street_name_size(); i++)
              begin_street_names->emplace_back(maneuver.begin_street_name(i));
            man->emplace("begin_street_names", std::move(begin_street_names));
          }

          // Time, length, and shape indexes
          man->emplace("time", static_cast<uint64_t>(maneuver.time()));
          man->emplace("length", json::fp_t{maneuver.length(), 3});
          man->emplace("begin_shape_index", static_cast<uint64_t>(maneuver.begin_shape_index()));
          man->emplace("end_shape_index", static_cast<uint64_t>(maneuver.end_shape_index()));

          // Portions toll and rough
          if (maneuver.portions_toll())
            man->emplace("toll", maneuver.portions_toll());
          if (maneuver.portions_unpaved())
            man->emplace("rough", maneuver.portions_unpaved());

          // Process sign
          if (maneuver.has_sign()) {
            auto sign = json::map({});

            // Process exit number
            if (maneuver.sign().exit_number_elements_size() > 0) {
              auto exit_number_elements = json::array({});
              for (int i = 0; i < maneuver.sign().exit_number_elements_size();
                  ++i) {
                auto exit_number_element = json::map({});

                // Add the exit number text
                exit_number_element->emplace(
                    "text", maneuver.sign().exit_number_elements(i).text());

                // Add the exit number consecutive count only if greater than zero
                if (maneuver.sign().exit_number_elements(i).consecutive_count() > 0) {
                  exit_number_element->emplace(
                      "consecutive_count",static_cast<uint64_t>(
                          maneuver.sign().exit_number_elements(i).consecutive_count()));
                }

                exit_number_elements->emplace_back(exit_number_element);
              }
              sign->emplace("exit_number_elements",
                            std::move(exit_number_elements));
            }

            // Process exit branch
            if (maneuver.sign().exit_branch_elements_size() > 0) {
              auto exit_branch_elements = json::array({});
              for (int i = 0; i < maneuver.sign().exit_branch_elements_size();
                  ++i) {
                auto exit_branch_element = json::map({});

                // Add the exit branch text
                exit_branch_element->emplace(
                    "text", maneuver.sign().exit_branch_elements(i).text());

                // Add the exit branch consecutive count only if greater than zero
                if (maneuver.sign().exit_branch_elements(i).consecutive_count() > 0) {
                  exit_branch_element->emplace(
                      "consecutive_count",static_cast<uint64_t>(
                          maneuver.sign().exit_branch_elements(i).consecutive_count()));
                }

                exit_branch_elements->emplace_back(exit_branch_element);
              }
              sign->emplace("exit_branch_elements",
                            std::move(exit_branch_elements));
            }

            // Process exit toward
            if (maneuver.sign().exit_toward_elements_size() > 0) {
              auto exit_toward_elements = json::array({});
              for (int i = 0; i < maneuver.sign().exit_toward_elements_size();
                  ++i) {
                auto exit_toward_element = json::map({});

                // Add the exit toward text
                exit_toward_element->emplace(
                    "text", maneuver.sign().exit_toward_elements(i).text());

                // Add the exit toward consecutive count only if greater than zero
                if (maneuver.sign().exit_toward_elements(i).consecutive_count() > 0) {
                  exit_toward_element->emplace(
                      "consecutive_count",static_cast<uint64_t>(
                          maneuver.sign().exit_toward_elements(i).consecutive_count()));
                }

                exit_toward_elements->emplace_back(exit_toward_element);
              }
              sign->emplace("exit_toward_elements",
                            std::move(exit_toward_elements));
            }

            // Process exit name
            if (maneuver.sign().exit_name_elements_size() > 0) {
              auto exit_name_elements = json::array({});
              for (int i = 0; i < maneuver.sign().exit_name_elements_size();
                  ++i) {
                auto exit_name_element = json::map({});

                // Add the exit name text
                exit_name_element->emplace(
                    "text", maneuver.sign().exit_name_elements(i).text());

                // Add the exit name consecutive count only if greater than zero
                if (maneuver.sign().exit_name_elements(i).consecutive_count() > 0) {
                  exit_name_element->emplace(
                      "consecutive_count",static_cast<uint64_t>(
                          maneuver.sign().exit_name_elements(i).consecutive_count()));
                }

                exit_name_elements->emplace_back(exit_name_element);
              }
              sign->emplace("exit_name_elements",
                            std::move(exit_name_elements));
            }

            man->emplace("sign", std::move(sign));
          }

          // Roundabout count
          if (maneuver.has_roundabout_exit_count()) {
            man->emplace("roundabout_exit_count", static_cast<uint64_t>(maneuver.roundabout_exit_count()));
          }

          // Depart and arrive instructions
          if (maneuver.has_depart_instruction()) {
            man->emplace("depart_instruction", maneuver.depart_instruction());
          }
          if (maneuver.has_verbal_depart_instruction()) {
            man->emplace("verbal_depart_instruction", maneuver.verbal_depart_instruction());
          }
          if (maneuver.has_arrive_instruction()) {
            man->emplace("arrive_instruction", maneuver.arrive_instruction());
          }
          if (maneuver.has_verbal_arrive_instruction()) {
            man->emplace("verbal_arrive_instruction", maneuver.verbal_arrive_instruction());
          }

          // Process transit route
          if (maneuver.has_transit_info()) {
            const auto& transit_info = maneuver.transit_info();
            auto json_transit_info = json::map({});

            if (transit_info.has_onestop_id()) {
              json_transit_info->emplace("onestop_id", transit_info.onestop_id());
              valhalla::midgard::logging::Log("transit_route_stopid::" + transit_info.onestop_id(), " [ANALYTICS] ");
            }
            if (transit_info.has_short_name()) {
              json_transit_info->emplace("short_name", transit_info.short_name());
            }
            if (transit_info.has_long_name()) {
              json_transit_info->emplace("long_name", transit_info.long_name());
            }
            if (transit_info.has_headsign()) {
              json_transit_info->emplace("headsign", transit_info.headsign());
            }
            if (transit_info.has_color()) {
              json_transit_info->emplace("color", static_cast<uint64_t>(transit_info.color()));
            }
            if (transit_info.has_text_color()) {
              json_transit_info->emplace("text_color", static_cast<uint64_t>(transit_info.text_color()));
            }
            if (transit_info.has_description()) {
              json_transit_info->emplace("description", transit_info.description());
            }
            if (transit_info.has_operator_onestop_id()) {
              json_transit_info->emplace("operator_onestop_id", transit_info.operator_onestop_id());
            }
            if (transit_info.has_operator_name()) {
              json_transit_info->emplace("operator_name", transit_info.operator_name());
            }
            if (transit_info.has_operator_url()) {
              json_transit_info->emplace("operator_url", transit_info.operator_url());
            }

            // Add transit stops
            if (transit_info.transit_stops().size() > 0) {
              auto json_transit_stops = json::array({});
              for (const auto& transit_stop : transit_info.transit_stops()) {
                auto json_transit_stop = json::map({});

                // type
                if (transit_stop.has_type()) {
                  if (transit_stop.type() == TransitPlatformInfo_Type_kStation) {
                    json_transit_stop->emplace("type", std::string("station"));
                  } else {
                    json_transit_stop->emplace("type", std::string("stop"));
                  }
                }

                // onestop_id - using the station onestop_id
                if (transit_stop.has_station_onestop_id()) {
                    json_transit_stop->emplace("onestop_id", transit_stop.station_onestop_id());
                    valhalla::midgard::logging::Log("transit_stopid::" + transit_stop.station_onestop_id(), " [ANALYTICS] ");
                }

                // name - using the station name
                if (transit_stop.has_station_name()) {
                    json_transit_stop->emplace("name", transit_stop.station_name());
                }

                // arrival_date_time
                if (transit_stop.has_arrival_date_time()) {
                    json_transit_stop->emplace("arrival_date_time", transit_stop.arrival_date_time());
                }

                // departure_date_time
                if (transit_stop.has_departure_date_time()) {
                    json_transit_stop->emplace("departure_date_time", transit_stop.departure_date_time());
                }

                // assumed_schedule
                if (transit_stop.has_assumed_schedule()) {
                    json_transit_stop->emplace("assumed_schedule", transit_stop.assumed_schedule());
                }

                // latitude and longitude
                if (transit_stop.has_ll()) {
                    json_transit_stop->emplace("lat", json::fp_t{transit_stop.ll().lat(), 6});
                    json_transit_stop->emplace("lon",json::fp_t{transit_stop.ll().lng(), 6});
                }

                json_transit_stops->emplace_back(json_transit_stop);

              }
              json_transit_info->emplace("transit_stops",
                                          std::move(json_transit_stops));
            }

            man->emplace("transit_info", std::move(json_transit_info));
          }

          if (maneuver.verbal_multi_cue())
            man->emplace("verbal_multi_cue", maneuver.verbal_multi_cue());

          // Travel mode
          auto mode_type = travel_mode_type(maneuver);
          man->emplace("travel_mode", mode_type.first);

          // Travel type
          man->emplace("travel_type", mode_type.second);

          //  man->emplace("hasGate", maneuver.);
          //  man->emplace("hasFerry", maneuver.);
          //“portionsTollNote” : “<portionsTollNote>”,
          //“portionsUnpavedNote” : “<portionsUnpavedNote>”,
          //“gateAccessRequiredNote” : “<gateAccessRequiredNote>”,
          //“checkFerryInfoNote” : “<checkFerryInfoNote>”
          maneuvers->emplace_back(man);

        }
        if (directions_leg.maneuver_size() > 0) {
          leg->emplace("maneuvers", maneuvers);
        }
        summary->emplace("time", static_cast<uint64_t>(directions_leg.summary().time()));
        summary->emplace("length", json::fp_t{directions_leg.summary().length(), 3});
        summary->emplace("min_lat", json::fp_t{directions_leg.summary().bbox().min_ll().lat(), 6});
        summary->emplace("min_lon",json::fp_t{directions_leg.summary().bbox().min_ll().lng(), 6});
        summary->emplace("max_lat", json::fp_t{directions_leg.summary().bbox().max_ll().lat(), 6});
        summary->emplace("max_lon",json::fp_t{directions_leg.summary().bbox().max_ll().lng(), 6});
        leg->emplace("summary",summary);
        leg->emplace("shape", directions_leg.shape());

        legs->emplace_back(leg);
      }
      return legs;
    }

    json::MapPtr serialize(const boost::optional<std::string>& id,
                   const valhalla::odin::DirectionsOptions& directions_options,
                   const std::list<valhalla::odin::TripDirections>& directions_legs) {

      //build up the json object
      auto json = json::map
      ({
        {"trip", json::map
          ({
            {"locations", locations(directions_legs)},
            {"summary", summary(directions_legs)},
            {"legs", legs(directions_legs)},
            {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
            {"status", static_cast<uint64_t>(0)}, //0 success
            {"units", std::string((directions_options.units() == valhalla::odin::DirectionsOptions::kKilometers) ? "kilometers" : "miles")},
            {"language", directions_options.language()}
          })
        }
      });
      if (id)
        json->emplace("id", *id);

      return json;
    }
  }



  void jsonToProtoLocation (const rapidjson::Value& json_location, Route::Location* proto_location) {
    // Set the lat
    auto lat_iter = json_location.FindMember("lat");
    if (lat_iter != json_location.MemberEnd()) {
      if (!lat_iter->value.IsFloat()) {
        throw std::runtime_error ("lat is not a float.");
      }
      proto_location->set_lat(lat_iter->value.GetFloat());
    }

    // Set the lon
    auto lon_iter = json_location.FindMember("lon");
    if (lon_iter != json_location.MemberEnd()) {
      if (!lon_iter->value.IsFloat()) {
        throw std::runtime_error ("lon is not a float.");
      }
      proto_location->set_lon(lon_iter->value.GetFloat());
    }

    // Set the type
    auto type_iter = json_location.FindMember("type");
    if (type_iter != json_location.MemberEnd()) {
      if (!type_iter->value.IsString()) {
        throw std::runtime_error ("type is not a string.");
      }
      proto_location->set_type(type_iter->value.GetString());
    }

    // Set the heading
    auto heading_iter = json_location.FindMember("heading");
    if (heading_iter != json_location.MemberEnd()) {
      if (!heading_iter->value.IsUint()) {
        throw std::runtime_error ("heading is not a Uint.");
      }
      proto_location->set_heading(heading_iter->value.GetUint());
    }

    // Set the name
    auto name_iter = json_location.FindMember("name");
    if (name_iter != json_location.MemberEnd()) {
      if (!name_iter->value.IsString()) {
        throw std::runtime_error ("name is not a string.");
      }
      proto_location->set_name(name_iter->value.GetString());
    }

    // Set the street
    auto street_iter = json_location.FindMember("street");
    if (street_iter != json_location.MemberEnd()) {
      if (!street_iter->value.IsString()) {
        throw std::runtime_error ("street is not a string.");
      }
      proto_location->set_street(street_iter->value.GetString());
    }

    // Set the city
    auto city_iter = json_location.FindMember("city");
    if (city_iter != json_location.MemberEnd()) {
      if (!city_iter->value.IsString()) {
        throw std::runtime_error ("city is not a string.");
      }
      proto_location->set_city(city_iter->value.GetString());
    }

    // Set the state
    auto state_iter = json_location.FindMember("state");
    if (state_iter != json_location.MemberEnd()) {
      if (!state_iter->value.IsString()) {
        throw std::runtime_error ("state is not a string.");
      }
      proto_location->set_state(state_iter->value.GetString());
    }

    // Set the postal_code
    auto postal_code_iter = json_location.FindMember("postal_code");
    if (postal_code_iter != json_location.MemberEnd()) {
      if (!postal_code_iter->value.IsString()) {
        throw std::runtime_error ("postal_code is not a string.");
      }
      proto_location->set_postal_code(postal_code_iter->value.GetString());
    }

    // Set the country
    auto country_iter = json_location.FindMember("country");
    if (country_iter != json_location.MemberEnd()) {
      if (!country_iter->value.IsString()) {
        throw std::runtime_error ("country is not a string.");
      }
      proto_location->set_country(country_iter->value.GetString());
    }

    // Set the date_time
    auto date_time_iter = json_location.FindMember("date_time");
    if (date_time_iter != json_location.MemberEnd()) {
      if (!date_time_iter->value.IsString()) {
        throw std::runtime_error ("date_time is not a string.");
      }
      proto_location->set_date_time(date_time_iter->value.GetString());
    }

    // Set the side_of_street
    auto side_of_street_iter = json_location.FindMember("side_of_street");
    if (side_of_street_iter != json_location.MemberEnd()) {
      if (!side_of_street_iter->value.IsString()) {
        throw std::runtime_error ("side_of_street is not a string.");
      }
      proto_location->set_side_of_street(side_of_street_iter->value.GetString());
    }

    // Set the original_index
    auto original_index_iter = json_location.FindMember("original_index");
    if (original_index_iter != json_location.MemberEnd()) {
      if (!original_index_iter->value.IsUint()) {
        throw std::runtime_error ("original_index is not a Uint.");
      }
      proto_location->set_original_index(original_index_iter->value.GetUint());
    }
  }

  void jsonToProtoSummary (const rapidjson::Value& json_summary, Route::Summary* proto_summary) {
    // Set the length
    auto length_iter = json_summary.FindMember("length");
    if (length_iter != json_summary.MemberEnd()) {
      if (!length_iter->value.IsFloat()) {
        throw std::runtime_error ("length is not a float.");
      }
      proto_summary->set_length(length_iter->value.GetFloat());
    }

    // Set the time
    auto time_iter = json_summary.FindMember("time");
    if (time_iter != json_summary.MemberEnd()) {
      if (!time_iter->value.IsUint()) {
        throw std::runtime_error ("time is not a Uint.");
      }
      proto_summary->set_time(time_iter->value.GetUint());
    }

    // Set the min_lat
    auto min_lat_iter = json_summary.FindMember("min_lat");
    if (min_lat_iter != json_summary.MemberEnd()) {
      if (!min_lat_iter->value.IsFloat()) {
        throw std::runtime_error ("min_lat is not a float.");
      }
      proto_summary->set_min_lat(min_lat_iter->value.GetFloat());
    }

    // Set the min_lon
    auto min_lon_iter = json_summary.FindMember("min_lon");
    if (min_lon_iter != json_summary.MemberEnd()) {
      if (!min_lon_iter->value.IsFloat()) {
        throw std::runtime_error ("min_lon is not a float.");
      }
      proto_summary->set_min_lon(min_lon_iter->value.GetFloat());
    }

    // Set the max_lat
    auto max_lat_iter = json_summary.FindMember("max_lat");
    if (max_lat_iter != json_summary.MemberEnd()) {
      if (!max_lat_iter->value.IsFloat()) {
        throw std::runtime_error ("max_lat is not a float.");
      }
      proto_summary->set_max_lat(max_lat_iter->value.GetFloat());
    }

    // Set the max_lon
    auto max_lon_iter = json_summary.FindMember("max_lon");
    if (max_lon_iter != json_summary.MemberEnd()) {
      if (!max_lon_iter->value.IsFloat()) {
        throw std::runtime_error ("max_lon is not a float.");
      }
      proto_summary->set_max_lon(max_lon_iter->value.GetFloat());
    }
  }

  void jsonToProtoElement (const rapidjson::Value& json_element, Route::Maneuver::Sign::Element* proto_element) {
    // Set the text
    auto text_iter = json_element.FindMember("text");
    if (text_iter != json_element.MemberEnd()) {
      if (!text_iter->value.IsString()) {
        throw std::runtime_error ("text is not a string.");
      }
      proto_element->set_text(text_iter->value.GetString());
    }

    // Set the consecutive_count
    auto consecutive_count_iter = json_element.FindMember("consecutive_count");
    if (consecutive_count_iter != json_element.MemberEnd()) {
      if (!consecutive_count_iter->value.IsUint()) {
        throw std::runtime_error ("consecutive_count is not a Uint.");
      }
      proto_element->set_consecutive_count(consecutive_count_iter->value.GetUint());
    }
  }

  void jsonToProtoSign (const rapidjson::Value& json_sign, Route::Maneuver::Sign* proto_sign) {
    // Set the exit_number_elements
    auto exit_number_elements_iter = json_sign.FindMember ("exit_number_elements");
    if (exit_number_elements_iter != json_sign.MemberEnd()) {
      if (!exit_number_elements_iter->value.IsArray()) {
        throw std::runtime_error ("exit_number_elements is not an array.");
      }
      auto proto_exit_number_elements = proto_sign->mutable_exit_number_elements();
      for (const auto& exit_number_element : exit_number_elements_iter->value.GetArray()) {
        if (!exit_number_element.IsObject()) {
          throw std::runtime_error ("exit_number_element is not an object.");
        }
        auto proto_exit_number_element = proto_exit_number_elements->Add();
        jsonToProtoElement(exit_number_element, proto_exit_number_element);
      }
    }

    // Set the exit_branch_elements
    auto exit_branch_elements_iter = json_sign.FindMember("exit_branch_elements");
    if (exit_branch_elements_iter != json_sign.MemberEnd()) {
      if (!exit_branch_elements_iter->value.IsArray()) {
        throw std::runtime_error ("exit_branch_element is not an array.");
      }
      auto proto_exit_branch_elements = proto_sign->mutable_exit_branch_elements();
      for (const auto& exit_branch_element : exit_branch_elements_iter->value.GetArray()) {
        if (!exit_branch_element.IsObject()) {
          throw std::runtime_error ("exit_branch_element is not an object.");
        }
        auto proto_exit_branch_element = proto_exit_branch_elements->Add();
        jsonToProtoElement(exit_branch_element, proto_exit_branch_element);
      }
    }

    // Set the exit_toward_elements
    auto exit_toward_elements_iter = json_sign.FindMember("exit_toward_elements");
    if (exit_toward_elements_iter != json_sign.MemberEnd()) {
      if (!exit_toward_elements_iter->value.IsArray()) {
        throw std::runtime_error ("exit_toward_element is not an array.");
      }
      auto proto_exit_toward_elements = proto_sign->mutable_exit_toward_elements();
      for (const auto& exit_toward_element : exit_toward_elements_iter->value.GetArray()) {
        if (!exit_toward_element.IsObject()) {
          throw std::runtime_error ("exit_toward_element is not an object.");
        }
        auto proto_exit_toward_element = proto_exit_toward_elements->Add();
        jsonToProtoElement(exit_toward_element, proto_exit_toward_element);
      }
    }

    // Set the exit_name_elements
    auto exit_name_elements_iter = json_sign.FindMember ("exit_name_elements");
    if (exit_name_elements_iter != json_sign.MemberEnd()) {
      if (!exit_name_elements_iter->value.IsArray()) {
        throw std::runtime_error ("exit_name_element is not an array.");
      }
      auto proto_exit_name_elements = proto_sign->mutable_exit_name_elements();
      for (const auto& exit_name_element : exit_name_elements_iter->value.GetArray()) {
        if (!exit_name_element.IsObject()) {
          throw std::runtime_error ("exit_name_element is not an object.");
        }
        auto proto_exit_name_element = proto_exit_name_elements->Add();
        jsonToProtoElement(exit_name_element, proto_exit_name_element);
      }
    }
  }

  void jsonToProtoTransitStop (const rapidjson::Value& json_transit_stop, Route::TransitStop* proto_transit_stop) {
    // Set the type
    auto type_iter = json_transit_stop.FindMember("type");
    if (type_iter != json_transit_stop.MemberEnd()) {
      if (!type_iter->value.IsString()) {
        throw std::runtime_error ("type is not a string.");
      }
      proto_transit_stop->set_type(type_iter->value.GetString());
    }

    // Set the onestop_id
    auto onestop_id_iter = json_transit_stop.FindMember("onestop_id");
    if (onestop_id_iter != json_transit_stop.MemberEnd()) {
      if (!onestop_id_iter->value.IsString()) {
        throw std::runtime_error ("onestop_id is not a string.");
      }
      proto_transit_stop->set_onestop_id(onestop_id_iter->value.GetString());
    }

    // Set the name
    auto name_iter = json_transit_stop.FindMember("name");
    if (name_iter != json_transit_stop.MemberEnd()) {
      if (!name_iter->value.IsString()) {
        throw std::runtime_error ("name is not a string.");
      }
      proto_transit_stop->set_name(name_iter->value.GetString());
    }

    // Set the arrival_date_time
    auto arrival_date_time_iter = json_transit_stop.FindMember("arrival_date_time");
    if (arrival_date_time_iter != json_transit_stop.MemberEnd()) {
      if (!arrival_date_time_iter->value.IsString()) {
        throw std::runtime_error ("arrival_date_time is not a string.");
      }
      proto_transit_stop->set_arrival_date_time(arrival_date_time_iter->value.GetString());
    }

    // Set the departure_date_time
    auto departure_date_time_iter = json_transit_stop.FindMember("departure_date_time");
    if (departure_date_time_iter != json_transit_stop.MemberEnd()) {
      if (!departure_date_time_iter->value.IsString()) {
        throw std::runtime_error ("departure_date_time is not a string.");
      }
      proto_transit_stop->set_departure_date_time(departure_date_time_iter->value.GetString());
    }

    // Set is_parent_stop
    auto is_parent_stop_iter = json_transit_stop.FindMember("is_parent_stop");
    if (is_parent_stop_iter != json_transit_stop.MemberEnd()) {
      if (!is_parent_stop_iter->value.IsBool()) {
        throw std::runtime_error ("is_parent_stop is not a bool.");
      }
      proto_transit_stop->set_is_parent_stop(is_parent_stop_iter->value.GetBool());
    }

    // Set assumed_schedule
    auto assumed_schedule_iter = json_transit_stop.FindMember("assumed_schedule");
    if (assumed_schedule_iter != json_transit_stop.MemberEnd()) {
      if (!assumed_schedule_iter->value.IsBool()) {
        throw std::runtime_error ("assumed_schedule is not a bool.");
      }
      proto_transit_stop->set_assumed_schedule(assumed_schedule_iter->value.GetBool());
    }

    // Set the lat
    auto lat_iter = json_transit_stop.FindMember("lat");
    if (lat_iter != json_transit_stop.MemberEnd()) {
      if (!lat_iter->value.IsFloat()) {
        throw std::runtime_error ("lat is not a float.");
      }
      proto_transit_stop->set_lat(lat_iter->value.GetFloat());
    }

    // Set the lon
    auto lon_iter = json_transit_stop.FindMember("lon");
    if (lon_iter != json_transit_stop.MemberEnd()) {
      if (!lon_iter->value.IsFloat()) {
        throw std::runtime_error ("lon is not a float.");
      }
      proto_transit_stop->set_lon(lon_iter->value.GetFloat());
    }
  }

  void jsonToProtoTransitInfo (const rapidjson::Value& json_transit_info, Route::TransitInfo* proto_transit_info) {
    // Set the onestop_id
    auto onestop_id_iter = json_transit_info.FindMember("onestop_id");
    if (onestop_id_iter != json_transit_info.MemberEnd()) {
      if (!onestop_id_iter->value.IsString()) {
        throw std::runtime_error ("onestop_id is not a string.");
      }
      proto_transit_info->set_onestop_id(onestop_id_iter->value.GetString());
    }

    // Set the short_name
    auto short_name_iter = json_transit_info.FindMember("short_name");
    if (short_name_iter != json_transit_info.MemberEnd()) {
      if (!short_name_iter->value.IsString()) {
        throw std::runtime_error ("short_name is not a string.");
      }
      proto_transit_info->set_short_name(short_name_iter->value.GetString());
    }

    // Set the long_name
    auto long_name_iter = json_transit_info.FindMember("long_name");
    if (long_name_iter != json_transit_info.MemberEnd()) {
      if (!long_name_iter->value.IsString()) {
        throw std::runtime_error ("long_name is not a string.");
      }
      proto_transit_info->set_long_name(long_name_iter->value.GetString());
    }

    // Set the headsign
    auto headsign_iter = json_transit_info.FindMember("headsign");
    if (headsign_iter != json_transit_info.MemberEnd()) {
      if (!headsign_iter->value.IsString()) {
        throw std::runtime_error ("headsign is not a string.");
      }
      proto_transit_info->set_headsign(headsign_iter->value.GetString());
    }

    // Set the color
    auto color_iter = json_transit_info.FindMember("color");
    if (color_iter != json_transit_info.MemberEnd()) {
      if (!color_iter->value.IsUint()) {
        throw std::runtime_error ("color is not a Uint.");
      }
      proto_transit_info->set_color(color_iter->value.GetUint());
    }

    // Set the text_color
    auto text_color_iter = json_transit_info.FindMember("text_color");
    if (text_color_iter != json_transit_info.MemberEnd()) {
      if (!text_color_iter->value.IsUint()) {
        throw std::runtime_error ("text_color is not a Uint.");
      }
      proto_transit_info->set_text_color(text_color_iter->value.GetUint());
    }

    // Set the description
    auto description_iter = json_transit_info.FindMember("description");
    if (description_iter != json_transit_info.MemberEnd()) {
      if (!description_iter->value.IsString()) {
        throw std::runtime_error ("description is not a string.");
      }
      proto_transit_info->set_description(description_iter->value.GetString());
    }

    // Set the operator_onestop_id
    auto operator_onestop_id_iter = json_transit_info.FindMember("operator_onestop_id");
    if (operator_onestop_id_iter != json_transit_info.MemberEnd()) {
      if (!operator_onestop_id_iter->value.IsString()) {
        throw std::runtime_error ("operator_onestop_id is not a string.");
      }
      proto_transit_info->set_operator_onestop_id(operator_onestop_id_iter->value.GetString());
    }

    // Set the operator_name
    auto operator_name_iter = json_transit_info.FindMember("operator_name");
    if (operator_name_iter != json_transit_info.MemberEnd()) {
      if (!operator_name_iter->value.IsString()) {
        throw std::runtime_error ("operator_name is not a string.");
      }
      proto_transit_info->set_operator_name(operator_name_iter->value.GetString());
    }

    // Set the operator_url
    auto operator_url_iter = json_transit_info.FindMember("operator_url");
    if (operator_url_iter != json_transit_info.MemberEnd()) {
      if (!operator_url_iter->value.IsString()) {
        throw std::runtime_error ("operator_url is not a string.");
      }
      proto_transit_info->set_operator_url(operator_url_iter->value.GetString());
    }

    // Set the transit_stops
    auto transit_stops_iter = json_transit_info.FindMember("transit_stops");
    if (transit_stops_iter != json_transit_info.MemberEnd()) {
      if (!transit_stops_iter->value.IsArray()) {
        throw std::runtime_error ("transit_stops is not an array.");
      }
      auto proto_transit_stops = proto_transit_info->mutable_transit_stops();
      for (const auto& transit_stop : transit_stops_iter->value.GetArray()) {
        if (!transit_stop.IsObject()) {
          throw std::runtime_error ("transit_stop is not an object.");
        }
        auto proto_transit_stop = proto_transit_stops->Add();
        jsonToProtoTransitStop(transit_stop, proto_transit_stop);
      }
    }
  }

  void jsonToProtoManeuver (const rapidjson::Value& json_maneuver, Route::Maneuver* proto_maneuver) {
    // Set the type
    auto type_iter = json_maneuver.FindMember("type");
    if (type_iter != json_maneuver.MemberEnd()) {
              if (!type_iter->value.IsUint()) {
                throw std::runtime_error ("type is not a Uint");
              }
      proto_maneuver->set_type(type_iter->value.GetUint());
    }

    // Set the instruction
    auto instruction_iter = json_maneuver.FindMember("instruction");
    if (instruction_iter != json_maneuver.MemberEnd()) {
              if (!instruction_iter->value.IsString()) {
                throw std::runtime_error ("instruction is not a string.");
              }
      proto_maneuver->set_instruction(instruction_iter->value.GetString());
    }

    // Set the street_names
    auto street_names_iter = json_maneuver.FindMember("street_names");
    if (street_names_iter != json_maneuver.MemberEnd()) {
      if (!street_names_iter->value.IsArray()) {
        throw std::runtime_error ("street_names is not an array.");
      }
      auto proto_street_names = proto_maneuver->mutable_street_names();
      for (const auto& street_name : street_names_iter->value.GetArray()) {
        if (!street_name.IsString()) {
          throw std::runtime_error ("street_name is not a string.");
        }
        auto proto_street_name = proto_street_names->Add();
        *proto_street_name = street_name.GetString();
      }
    }

    // Set the length
    auto length_iter = json_maneuver.FindMember("length");
    if (length_iter != json_maneuver.MemberEnd()) {
      if (!length_iter->value.IsFloat()) {
        throw std::runtime_error ("length is not a float.");
      }
      proto_maneuver->set_length(length_iter->value.GetFloat());
    }

    // Set the time
    auto time_iter = json_maneuver.FindMember("time");
    if (time_iter != json_maneuver.MemberEnd()) {
      if (!time_iter->value.IsUint()) {
        throw std::runtime_error ("time is not a Uint.");
      }
      proto_maneuver->set_time(time_iter->value.GetUint());
    }

    // Set the begin_cardinal_direction
    auto begin_cardinal_direction_iter = json_maneuver.FindMember("begin_cardinal_direction");
    if (begin_cardinal_direction_iter != json_maneuver.MemberEnd()) {
      if (!begin_cardinal_direction_iter->value.IsString()) {
        throw std::runtime_error ("begin_cardinal_direction is not a string.");
      }
      proto_maneuver->set_begin_cardinal_direction(begin_cardinal_direction_iter->value.GetString());
    }

    // Set the begin_heading
    auto begin_heading_iter = json_maneuver.FindMember("begin_heading");
    if (begin_heading_iter != json_maneuver.MemberEnd()) {
      if (!begin_heading_iter->value.IsUint()) {
        throw std::runtime_error ("begin_heading is not a Uint.");
      }
      proto_maneuver->set_begin_heading(begin_heading_iter->value.GetUint());
    }

    // Set the begin_shape_index
    auto begin_shape_index_iter = json_maneuver.FindMember("begin_shape_index");
    if (begin_shape_index_iter != json_maneuver.MemberEnd()) {
      if (!begin_shape_index_iter->value.IsUint()) {
        throw std::runtime_error ("begin_shape_index is not a Uint.");
      }
      proto_maneuver->set_begin_shape_index(begin_shape_index_iter->value.GetUint());
    }

    // Set the end_shape_index
    auto end_shape_index_iter = json_maneuver.FindMember("end_shape_index");
    if (end_shape_index_iter != json_maneuver.MemberEnd()) {
      if (!end_shape_index_iter->value.IsUint()) {
        throw std::runtime_error ("end_shape_index is not a Uint.");
      }
      proto_maneuver->set_end_shape_index(end_shape_index_iter->value.GetUint());
    }

    // Set toll
    auto toll_iter = json_maneuver.FindMember("toll");
    if (toll_iter != json_maneuver.MemberEnd()) {
      if (!toll_iter->value.IsBool()) {
        throw std::runtime_error ("toll is not a bool.");
      }
      proto_maneuver->set_toll(toll_iter->value.GetBool());
    }

    // Set rough
    auto rough_iter = json_maneuver.FindMember("rough");
    if (rough_iter != json_maneuver.MemberEnd()) {
      if (!rough_iter->value.IsBool()) {
        throw std::runtime_error ("rough is not a bool.");
      }
      proto_maneuver->set_rough(rough_iter->value.GetBool());
    }

    // Set the verbal_transition_alert_instruction
    auto verbal_transition_alert_instruction_iter = json_maneuver.FindMember("verbal_transition_alert_instruction");
    if (verbal_transition_alert_instruction_iter != json_maneuver.MemberEnd()) {
      if (!verbal_transition_alert_instruction_iter->value.IsString()) {
        throw std::runtime_error ("verbal_transition_alert_instruction is not a string.");
      }
      proto_maneuver->set_verbal_transition_alert_instruction(verbal_transition_alert_instruction_iter->value.GetString());
    }

    // Set the verbal_pre_transition_instruction
    auto verbal_pre_transition_instruction_iter = json_maneuver.FindMember("verbal_pre_transition_instruction");
    if (verbal_pre_transition_instruction_iter != json_maneuver.MemberEnd()) {
      if (!verbal_pre_transition_instruction_iter->value.IsString()) {
        throw std::runtime_error ("verbal_pre_transition_instruction is not a string.");
      }
      proto_maneuver->set_verbal_pre_transition_instruction(verbal_pre_transition_instruction_iter->value.GetString());
    }

    // Set the verbal_post_transition_instruction
    auto verbal_post_transition_instruction_iter = json_maneuver.FindMember("verbal_post_transition_instruction");
    if (verbal_post_transition_instruction_iter != json_maneuver.MemberEnd()) {
      if (!verbal_post_transition_instruction_iter->value.IsString()) {
        throw std::runtime_error ("verbal_post_transition_instruction is not a string.");
      }
      proto_maneuver->set_verbal_post_transition_instruction(verbal_post_transition_instruction_iter->value.GetString());
    }

    // Set the begin_street_names
    auto begin_street_names_iter = json_maneuver.FindMember("begin_street_names");
    if (begin_street_names_iter != json_maneuver.MemberEnd()) {
      if (!begin_street_names_iter->value.IsArray()) {
        throw std::runtime_error ("begin_street_names is not an array.");
      }
      auto proto_begin_street_names = proto_maneuver->mutable_begin_street_names();
      for (const auto& begin_street_name : begin_street_names_iter->value.GetArray()) {
        if (!begin_street_name.IsString()) {
          throw std::runtime_error ("begin_street_name is not a string.");
        }
        auto proto_begin_street_name = proto_begin_street_names->Add();
        *proto_begin_street_name = begin_street_name.GetString();
      }
    }

    // Set the sign
    auto sign_iter = json_maneuver.FindMember ("sign");
    if (sign_iter != json_maneuver.MemberEnd()) {
      if (!sign_iter->value.IsObject()) {
        throw std::runtime_error ("sign is not an object.");
      }
      jsonToProtoSign (sign_iter->value, proto_maneuver->mutable_sign());
    }

    // Set the roundabout_exit_count
    auto roundabout_exit_count_iter = json_maneuver.FindMember("roundabout_exit_count");
    if (roundabout_exit_count_iter != json_maneuver.MemberEnd()) {
      if (!roundabout_exit_count_iter->value.IsUint()) {
        throw std::runtime_error ("roundabout_exit_count is not a Uint.");
      }
      proto_maneuver->set_roundabout_exit_count(roundabout_exit_count_iter->value.GetUint());
    }

    // Set the depart_instruction
    auto depart_instruction_iter = json_maneuver.FindMember("depart_instruction");
    if (depart_instruction_iter != json_maneuver.MemberEnd()) {
      if (!depart_instruction_iter->value.IsString()) {
        throw std::runtime_error ("depart_instruction is not a string.");
      }
      proto_maneuver->set_depart_instruction(depart_instruction_iter->value.GetString());
    }

    // Set the verbal_depart_instruction
    auto verbal_depart_instruction_iter = json_maneuver.FindMember("verbal_depart_instruction");
    if (verbal_depart_instruction_iter != json_maneuver.MemberEnd()) {
      if (!verbal_depart_instruction_iter->value.IsString()) {
        throw std::runtime_error ("verbal_depart_instruction is not a string.");
      }
      proto_maneuver->set_verbal_depart_instruction(verbal_depart_instruction_iter->value.GetString());
    }

    // Set the arrive_instruction
    auto arrive_instruction_iter = json_maneuver.FindMember("arrive_instruction");
    if (arrive_instruction_iter != json_maneuver.MemberEnd()) {
      if (!arrive_instruction_iter->value.IsString()) {
        throw std::runtime_error ("arrive_instruction is not a string.");
      }
      proto_maneuver->set_arrive_instruction(arrive_instruction_iter->value.GetString());
    }

    // Set the verbal_arrive_instruction
    auto verbal_arrive_instruction_iter = json_maneuver.FindMember("verbal_arrive_instruction");
    if (verbal_arrive_instruction_iter != json_maneuver.MemberEnd()) {
      if (!verbal_arrive_instruction_iter->value.IsString()) {
        throw std::runtime_error ("verbal_arrive_instruction is not a string.");
      }
      proto_maneuver->set_verbal_arrive_instruction(verbal_arrive_instruction_iter->value.GetString());
    }

    // Set the transit_info
    auto transit_info_iter = json_maneuver.FindMember("transit_info");
    if (transit_info_iter != json_maneuver.MemberEnd()) {
      if (!transit_info_iter->value.IsObject()) {
        throw std::runtime_error ("transit_info is not an object.");
      }
      jsonToProtoTransitInfo (transit_info_iter->value, proto_maneuver->mutable_transit_info());
    }

    // Set verbal_multi_cue
    auto verbal_multi_cue_iter = json_maneuver.FindMember("verbal_multi_cue");
    if (verbal_multi_cue_iter != json_maneuver.MemberEnd()) {
      if (!verbal_multi_cue_iter->value.IsBool()) {
        throw std::runtime_error ("verbal_multi_cue is not a bool.");
      }
      proto_maneuver->set_verbal_multi_cue(verbal_multi_cue_iter->value.GetBool());
    }

    // Set the travel_mode
    auto travel_mode_iter = json_maneuver.FindMember("travel_mode");
    if (travel_mode_iter != json_maneuver.MemberEnd()) {
      if (!travel_mode_iter->value.IsString()) {
        throw std::runtime_error ("travel_mode is not a string.");
      }
      proto_maneuver->set_travel_mode(travel_mode_iter->value.GetString());
    }

    // Set the travel_type
    auto travel_type_iter = json_maneuver.FindMember("travel_type");
    if (travel_type_iter != json_maneuver.MemberEnd()) {
      if (!travel_type_iter->value.IsString()) {
        throw std::runtime_error ("travel_type is not a string.");
      }
      proto_maneuver->set_travel_type(travel_type_iter->value.GetString());
    }
  }

  void jsonToProtoLeg (const rapidjson::Value& json_leg, Route::Leg* proto_leg) {
    // Set the summary
    auto summary_iter = json_leg.FindMember("summary");
    if (summary_iter != json_leg.MemberEnd()) {
      if (!summary_iter->value.IsObject()) {
        throw std::runtime_error ("summary is not an object.");
      }
      jsonToProtoSummary (summary_iter->value, proto_leg->mutable_summary());
    }

    // Set the maneuvers
    auto maneuvers_iter = json_leg.FindMember("maneuvers");
    if(maneuvers_iter != json_leg.MemberEnd()) {
      if (!maneuvers_iter->value.IsArray()) {
        throw std::runtime_error ("maneuvers is not an array.");
      }
      auto proto_maneuvers = proto_leg->mutable_maneuvers();
      for (const auto& maneuver : maneuvers_iter->value.GetArray()) {
        if (!maneuver.IsObject()) {
          throw std::runtime_error ("maneuver is not an object.");
        }
        auto proto_maneuver = proto_maneuvers->Add();
        jsonToProtoManeuver (maneuver, proto_maneuver);
      }
    }

    // Set the shape
    auto shape_iter = json_leg.FindMember("shape");
    if (shape_iter != json_leg.MemberEnd()) {
      if (!shape_iter->value.IsString()) {
        throw std::runtime_error ("shape is not a string.");
      }
      proto_leg->set_shape(shape_iter->value.GetString());
    }
  }
}


namespace valhalla {
  namespace tyr {

    json::MapPtr serializeDirections(ACTION_TYPE action, const boost::property_tree::ptree& request,
        const std::list<TripDirections>& legs) {
      //see if we can get some options
      valhalla::odin::DirectionsOptions directions_options;
      auto options = request.get_child_optional("directions_options");
      if(options)
        directions_options = valhalla::odin::GetDirectionsOptions(*options);

      //serialize them
      if(action == VIAROUTE)
        return osrm_serializers::serialize(directions_options, legs);
      else
        return valhalla_serializers::serialize(request.get_optional<std::string>("id"), directions_options, legs);
    }

    void jsonToProtoRoute (const std::string& json_route, Route& proto_route) {
      rapidjson::Document d;
      d.Parse (json_route.c_str());
      if (d.HasParseError())
        throw std::runtime_error ("String to document parsing failed.");

      // Grab the trip object from JSON
      auto json_trip_iter = d.FindMember("trip");
      if (json_trip_iter == d.MemberEnd()) {
        return;
      } else if (!json_trip_iter->value.IsObject()) {
        throw std::runtime_error ("trip is not an object.");
      }

      if (proto_route.has_trip())
        proto_route.clear_trip();

      // Get the empty trip object and start setting it's fields
      Route::Trip* proto_trip = proto_route.mutable_trip();


      // Set the locations
      auto locations_iter = json_trip_iter->value.FindMember("locations");
      if (locations_iter != json_trip_iter->value.MemberEnd()) {
        if (!locations_iter->value.IsArray()) {
          throw std::runtime_error ("locations is not an array.");
        }
        auto proto_locations = proto_trip->mutable_locations();
        for (const auto& loc : locations_iter->value.GetArray()) {
          if (!loc.IsObject()) {
            throw std::runtime_error ("location is not an object.");
          }
          auto proto_loc = proto_locations->Add();
          jsonToProtoLocation (loc, proto_loc);
        }
      }

      // Set the summary
      auto summary_iter = json_trip_iter->value.FindMember("summary");
      if (summary_iter != json_trip_iter->value.MemberEnd()) {
        if (!summary_iter->value.IsObject()) {
          throw std::runtime_error ("summary is not an object.");
        }
        jsonToProtoSummary (summary_iter->value, proto_trip->mutable_summary());
      }

      // Set the legs
      auto legs_iter = json_trip_iter->value.FindMember("legs");
      if (legs_iter != json_trip_iter->value.MemberEnd()) {
        if (!legs_iter->value.IsArray()) {
          throw std::runtime_error ("legs is not an array.");
        }
        auto proto_legs = proto_trip->mutable_legs();
        for (const auto& leg: legs_iter->value.GetArray()) {
          if (!leg.IsObject()) {
            throw std::runtime_error ("leg is not an object.");
          }
          auto proto_leg = proto_legs->Add();
          jsonToProtoLeg (leg, proto_leg);
        }
      }

      // Set the status_message
      auto status_message_iter = json_trip_iter->value.FindMember("status_message");
      if (status_message_iter != json_trip_iter->value.MemberEnd()) {
        if (!status_message_iter->value.IsString()) {
          throw std::runtime_error ("status_message is not a string.");
        }
        proto_trip->set_status_message(status_message_iter->value.GetString());
      }

      // Set the status
      auto status_iter = json_trip_iter->value.FindMember("status");
      if (status_iter != json_trip_iter->value.MemberEnd()) {
        if (!status_iter->value.IsUint()) {
          throw std::runtime_error ("status is not a Uint.");
        }
        proto_trip->set_status(status_iter->value.GetUint());
      }

      // Set the units
      auto units_iter = json_trip_iter->value.FindMember("units");
      if (units_iter != json_trip_iter->value.MemberEnd()) {
        if (!units_iter->value.IsString()) {
          throw std::runtime_error ("units is not a string.");
        }
        proto_trip->set_units(units_iter->value.GetString());
      }

      // Set the language
      auto language_iter = json_trip_iter->value.FindMember("language");
      if (language_iter != json_trip_iter->value.MemberEnd()) {
        if (!language_iter->value.IsString()) {
          throw std::runtime_error ("language is not a string.");
        }
        proto_trip->set_language(language_iter->value.GetString());
      }

      // Set the id
      auto id_iter = json_trip_iter->value.FindMember("id");
      if (id_iter != json_trip_iter->value.MemberEnd()) {
        if (!id_iter->value.IsString()) {
          throw std::runtime_error ("id is not a string.");
        }
        proto_trip->set_id(id_iter->value.GetString());
      }
    }

  }
}
