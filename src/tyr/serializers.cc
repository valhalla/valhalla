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
using namespace valhalla::service;
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
                  if (transit_stop.type() == TripDirections_TransitStop_Type_kStation) {
                    json_transit_stop->emplace("type", std::string("station"));
                  } else {
                    json_transit_stop->emplace("type", std::string("stop"));
                  }
                }

                // onestop_id
                if (transit_stop.has_onestop_id()) {
                    json_transit_stop->emplace("onestop_id", transit_stop.onestop_id());
                    valhalla::midgard::logging::Log("transit_stopid::" + transit_stop.onestop_id(), " [ANALYTICS] ");
                }

                // name
                if (transit_stop.has_name()) {
                    json_transit_stop->emplace("name", transit_stop.name());
                }

                // arrival_date_time
                if (transit_stop.has_arrival_date_time()) {
                    json_transit_stop->emplace("arrival_date_time", transit_stop.arrival_date_time());
                }

                // departure_date_time
                if (transit_stop.has_departure_date_time()) {
                    json_transit_stop->emplace("departure_date_time", transit_stop.departure_date_time());
                }

                // is_parent_stop
                if (transit_stop.has_is_parent_stop()) {
                    json_transit_stop->emplace("is_parent_stop", transit_stop.is_parent_stop());
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
    for (const auto& member : json_location.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the lat
      if (member_name == "lat") {
        proto_location->set_lat(member.value.GetFloat());
      }

      // Set the lon
      else if (member_name == "lon") {
        proto_location->set_lon(member.value.GetFloat());
      }

      // Set the type
      else if (member_name == "type") {
        proto_location->set_type(member.value.GetString());
      }

      // Set the heading
      else if (member_name == "heading") {
        proto_location->set_heading(member.value.GetUint());
      }

      // Set the name
      else if (member_name == "name") {
        proto_location->set_name(member.value.GetString());
      }

      // Set the street
      else if (member_name == "street") {
        proto_location->set_street(member.value.GetString());
      }

      // Set the city
      else if (member_name == "city") {
        proto_location->set_city(member.value.GetString());
      }

      // Set the state
      else if (member_name == "state") {
        proto_location->set_state(member.value.GetString());
      }

      // Set the postal_code
      else if (member_name == "postal_code") {
        proto_location->set_postal_code(member.value.GetString());
      }

      // Set the country
      else if (member_name == "country") {
        proto_location->set_country(member.value.GetString());
      }

      // Set the date_time
      else if (member_name == "date_time") {
        proto_location->set_date_time(member.value.GetString());
      }

      // Set the side_of_street
      else if (member_name == "side_of_street") {
        proto_location->set_side_of_street(member.value.GetString());
      }

      // Set the original_index
      else if (member_name == "original_index") {
        proto_location->set_original_index(member.value.GetUint());
      }
    }
  }

  void jsonToProtoSummary (const rapidjson::Value& json_summary, Route::Summary* proto_summary) {
    for (const auto& member : json_summary.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the length
      if (member_name == "length") {
        proto_summary->set_length(member.value.GetFloat());
      }

      // Set the time
      else if (member_name == "time") {
        proto_summary->set_time(member.value.GetUint());
      }

      // Set the min_lat
      else if (member_name == "min_lat") {
        proto_summary->set_min_lat(member.value.GetFloat());
      }

      // Set the min_lon
      else if (member_name == "min_lon") {
        proto_summary->set_min_lon(member.value.GetFloat());
      }

      // Set the max_lat
      else if (member_name == "max_lat") {
        proto_summary->set_max_lat(member.value.GetFloat());
      }

      // Set the max_lon
      else if (member_name == "max_lon") {
        proto_summary->set_max_lon(member.value.GetFloat());
      }
    }
  }

  void jsonToProtoElement (const rapidjson::Value& json_element, Route::Maneuver::Sign::Element* proto_element) {
    for (const auto& member : json_element.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the text
      if (member_name == "text") {
        proto_element->set_text(member.value.GetString());
      }

      // Set the consecutive_count
      else if (member_name == "consecutive_count") {
        proto_element->set_consecutive_count(member.value.GetUint());
      }
    }
  }

  void jsonToProtoSign (const rapidjson::Value& json_sign, Route::Maneuver::Sign* proto_sign) {
    for (const auto& member : json_sign.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the exit_number_elements
      if (member_name == "exit_number_elements") {
        auto proto_exit_number_elements = proto_sign->mutable_exit_number_elements();
        for (const auto& exit_number_element : member.value.GetArray()) {
          auto proto_exit_number_element = proto_exit_number_elements->Add();
          jsonToProtoElement(exit_number_element, proto_exit_number_element);
        }
      }

      // Set the exit_branch_elements
      if (member_name == "exit_branch_elements") {
        auto proto_exit_branch_elements = proto_sign->mutable_exit_branch_elements();
        for (const auto& exit_branch_element : member.value.GetArray()) {
          auto proto_exit_branch_element = proto_exit_branch_elements->Add();
          jsonToProtoElement(exit_branch_element, proto_exit_branch_element);
        }
      }

      // Set the exit_toward_elements
      if (member_name == "exit_toward_elements") {
        auto proto_exit_toward_elements = proto_sign->mutable_exit_toward_elements();
        for (const auto& exit_toward_element : member.value.GetArray()) {
          auto proto_exit_toward_element = proto_exit_toward_elements->Add();
          jsonToProtoElement(exit_toward_element, proto_exit_toward_element);
        }
      }

      // Set the exit_name_elements
      if (member_name == "exit_name_elements") {
        auto proto_exit_name_elements = proto_sign->mutable_exit_name_elements();
        for (const auto& exit_name_element : member.value.GetArray()) {
          auto proto_exit_name_element = proto_exit_name_elements->Add();
          jsonToProtoElement(exit_name_element, proto_exit_name_element);
        }
      }
    }
  }

  void jsonToProtoTransitStop (const rapidjson::Value& json_transit_stop, Route::TransitStop* proto_transit_stop) {
    for (const auto& member : json_transit_stop.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the type
      if (member_name == "type") {
        proto_transit_stop->set_type(member.value.GetString());
      }

      // Set the onestop_id
      else if (member_name == "onestop_id") {
        proto_transit_stop->set_onestop_id(member.value.GetString());
      }

      // Set the name
      else if (member_name == "name") {
        proto_transit_stop->set_name(member.value.GetString());
      }

      // Set the arrival_date_time
      else if (member_name == "arrival_date_time") {
        proto_transit_stop->set_arrival_date_time(member.value.GetString());
      }

      // Set the departure_date_time
      else if (member_name == "departure_date_time") {
        proto_transit_stop->set_departure_date_time(member.value.GetString());
      }

      // Set is_parent_stop
      else if (member_name == "is_parent_stop") {
        proto_transit_stop->set_is_parent_stop(member.value.GetBool());
      }

      // Set assumed_schedule
      else if (member_name == "assumed_schedule") {
        proto_transit_stop->set_assumed_schedule(member.value.GetBool());
      }

      // Set the lat
      else if (member_name == "lat") {
        proto_transit_stop->set_lat(member.value.GetFloat());
      }

      // Set the lon
      else if (member_name == "lon") {
        proto_transit_stop->set_lon(member.value.GetFloat());
      }
    }
  }

  void jsonToProtoTransitInfo (const rapidjson::Value& json_transit_info, Route::TransitInfo* proto_transit_info) {
    for (const auto& member : json_transit_info.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the onestop_id
      if (member_name == "onestop_id") {
        proto_transit_info->set_onestop_id(member.value.GetString());
      }

      // Set the short_name
      else if (member_name == "short_name") {
        proto_transit_info->set_short_name(member.value.GetString());
      }

      // Set the long_name
      else if (member_name == "long_name") {
        proto_transit_info->set_long_name(member.value.GetString());
      }

      // Set the headsign
      else if (member_name == "headsign") {
        proto_transit_info->set_headsign(member.value.GetString());
      }

      // Set the color
      else if (member_name == "color") {
        proto_transit_info->set_color(member.value.GetUint());
      }

      // Set the text_color
      else if (member_name == "text_color") {
        proto_transit_info->set_text_color(member.value.GetUint());
      }

      // Set the description
      else if (member_name == "description") {
        proto_transit_info->set_description(member.value.GetString());
      }

      // Set the operator_onestop_id
      else if (member_name == "operator_onestop_id") {
        proto_transit_info->set_operator_onestop_id(member.value.GetString());
      }

      // Set the operator_name
      else if (member_name == "operator_name") {
        proto_transit_info->set_operator_name(member.value.GetString());
      }

      // Set the operator_url
      else if (member_name == "operator_url") {
        proto_transit_info->set_operator_url(member.value.GetString());
      }

      // Set the transit_stops
      else if (member_name == "transit_stops") {
        auto proto_transit_stops = proto_transit_info->mutable_transit_stops();
        for (const auto& transit_stop : member.value.GetArray()) {
          auto proto_transit_stop = proto_transit_stops->Add();
          jsonToProtoTransitStop(transit_stop, proto_transit_stop);
        }
      }
    }
  }

  void jsonToProtoManeuver (const rapidjson::Value& json_maneuver, Route::Maneuver* proto_maneuver) {
    for (const auto& member : json_maneuver.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the type
      if (member_name == "type") {
        proto_maneuver->set_type(member.value.GetUint());
      }

      // Set the instruction
      else if (member_name == "instruction") {
        proto_maneuver->set_instruction(member.value.GetString());
      }

      // Set the street_names
      else if (member_name == "street_names") {
        auto proto_street_names = proto_maneuver->mutable_street_names();
        for (const auto& street_name : member.value.GetArray()) {
          auto proto_street_name = proto_street_names->Add();
          *proto_street_name = street_name.GetString();
        }
      }

      // Set the length
      else if (member_name == "length") {
        proto_maneuver->set_length(member.value.GetFloat());
      }

      // Set the time
      else if (member_name == "time") {
        proto_maneuver->set_time(member.value.GetUint());
      }

      // Set the begin_cardinal_direction
      else if (member_name == "begin_cardinal_direction") {
        proto_maneuver->set_begin_cardinal_direction(member.value.GetString());
      }

      // Set the begin_heading
      else if (member_name == "begin_heading") {
        proto_maneuver->set_begin_heading(member.value.GetUint());
      }

      // Set the begin_shape_index
      else if (member_name == "begin_shape_index") {
        proto_maneuver->set_begin_shape_index(member.value.GetUint());
      }

      // Set the end_shape_index
      else if (member_name == "end_shape_index") {
        proto_maneuver->set_end_shape_index(member.value.GetUint());
      }

      // Set toll
      else if (member_name == "toll") {
        proto_maneuver->set_toll(member.value.GetBool());
      }

      // Set rough
      else if (member_name == "rough") {
        proto_maneuver->set_rough(member.value.GetBool());
      }

      // Set the verbal_transition_alert_instruction
      else if (member_name == "verbal_transition_alert_instruction") {
        proto_maneuver->set_verbal_transition_alert_instruction(member.value.GetString());
      }

      // Set the verbal_pre_transition_instruction
      else if (member_name == "verbal_pre_transition_instruction") {
        proto_maneuver->set_verbal_pre_transition_instruction(member.value.GetString());
      }

      // Set the verbal_post_transition_instruction
      else if (member_name == "verbal_post_transition_instruction") {
        proto_maneuver->set_verbal_post_transition_instruction(member.value.GetString());
      }

      // Set the begin_street_names
      else if (member_name == "begin_street_names") {
        auto proto_begin_street_names = proto_maneuver->mutable_begin_street_names();
        for (const auto& begin_street_name : member.value.GetArray()) {
          auto proto_begin_street_name = proto_begin_street_names->Add();
          *proto_begin_street_name = begin_street_name.GetString();
        }
      }

      // Set the sign
      else if (member_name == "sign") {
        jsonToProtoSign (member.value, proto_maneuver->mutable_sign());
      }

      // Set the roundabout_exit_count
      else if (member_name == "roundabout_exit_count") {
        proto_maneuver->set_roundabout_exit_count(member.value.GetUint());
      }

      // Set the depart_instruction
      else if (member_name == "depart_instruction") {
        proto_maneuver->set_depart_instruction(member.value.GetString());
      }

      // Set the verbal_depart_instruction
      else if (member_name == "verbal_depart_instruction") {
        proto_maneuver->set_verbal_depart_instruction(member.value.GetString());
      }

      // Set the arrive_instruction
      else if (member_name == "arrive_instruction") {
        proto_maneuver->set_arrive_instruction(member.value.GetString());
      }

      // Set the verbal_arrive_instruction
      else if (member_name == "verbal_arrive_instruction") {
        proto_maneuver->set_verbal_arrive_instruction(member.value.GetString());
      }

      // Set the transit_info
      else if (member_name == "transit_info") {
        jsonToProtoTransitInfo (member.value, proto_maneuver->mutable_transit_info());
      }

      // Set verbal_multi_cue
      else if (member_name == "verbal_multi_cue") {
        proto_maneuver->set_verbal_multi_cue(member.value.GetBool());
      }

      // Set the travel_mode
      else if (member_name == "travel_mode") {
        proto_maneuver->set_travel_mode(member.value.GetString());
      }

      // Set the travel_type
      else if (member_name == "travel_type") {
        proto_maneuver->set_travel_type(member.value.GetString());
      }
    }
  }

  void jsonToProtoLeg (const rapidjson::Value& json_leg, Route::Leg* proto_leg) {
    for (const auto& member : json_leg.GetObject()) {
      std::string member_name = member.name.GetString();

      // Set the summary
      if (member_name == "summary") {
        jsonToProtoSummary (member.value, proto_leg->mutable_summary());
      }

      // Set the maneuvers
      if(member_name == "maneuvers") {
        auto proto_maneuvers = proto_leg->mutable_maneuvers();
        for (const auto& maneuver : member.value.GetArray()) {
          auto proto_maneuver = proto_maneuvers->Add();
          jsonToProtoManeuver (maneuver, proto_maneuver);
        }
      }

      // Set the shape
      if (member_name == "shape") {
        proto_leg->set_shape(member.value.GetString());
      }
    }
  }
}


namespace valhalla {
  namespace tyr {

    json::MapPtr serialize(service::ACTION_TYPE action, const boost::property_tree::ptree& request,
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

      // Grab the trip object from JSON
      auto json_trip = GetOptionalFromRapidJson<rapidjson::Value::Object> (d, "/trip");
      if (!json_trip) {
        return;
      }

      if (proto_route.has_trip())
        proto_route.clear_trip();

      // Get the empty trip object and start setting it's fields
      Route::Trip* proto_trip = proto_route.mutable_trip();

      for (const auto& member : *json_trip) {
        std::string member_name = member.name.GetString();

        // Set the locations
        if (member_name == "locations") {
          auto proto_locations = proto_trip->mutable_locations();
          for (const auto& loc : member.value.GetArray()) {
            auto proto_loc = proto_locations->Add();
            jsonToProtoLocation (loc, proto_loc);
          }
        }

        // Set the summary
        else if (member_name == "summary") {
          jsonToProtoSummary (member.value, proto_trip->mutable_summary());
        }

        // Set the legs
        else if (member_name == "legs") {
          auto proto_legs = proto_trip->mutable_legs();
          for (const auto& leg: member.value.GetArray()) {
            auto proto_leg = proto_legs->Add();
            jsonToProtoLeg (leg, proto_leg);
          }
        }

        // Set the status_message
        else if (member_name == "status_message") {
          proto_trip->set_status_message(member.value.GetString());
        }

        // Set the status
        else if (member_name == "status") {
          proto_trip->set_status(member.value.GetUint());
        }

        // Set the units
        else if (member_name == "units") {
          proto_trip->set_units(member.value.GetString());
        }

        // Set the language
        else if (member_name == "language") {
          proto_trip->set_language(member.value.GetString());
        }

        // Set the id
        else if (member_name == "id") {
          proto_trip->set_id(member.value.GetString());
        }
      }
    }

  }
}
