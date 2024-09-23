#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

#include "baldr/admin.h"
#include "baldr/datetime.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "baldr/landmark.h"
#include "baldr/signinfo.h"
#include "baldr/tilehierarchy.h"
#include "baldr/time_info.h"
#include "baldr/timedomain.h"
#include "meili/match_result.h"
#include "midgard/elevation_encoding.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "proto/common.pb.h"
#include "sif/costconstants.h"
#include "sif/recost.h"
#include "thor/triplegbuilder.h"
#include "triplegbuilder_utils.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

using LinguisticMap = std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>;

constexpr uint8_t kNotTagged = 0;
constexpr uint8_t kTunnelTag = static_cast<uint8_t>(baldr::TaggedValue::kTunnel);
constexpr uint8_t kBridgeTag = static_cast<uint8_t>(baldr::TaggedValue::kBridge);

uint32_t
GetAdminIndex(const AdminInfo& admin_info,
              std::unordered_map<AdminInfo, uint32_t, AdminInfo::AdminInfoHasher>& admin_info_map,
              std::vector<AdminInfo>& admin_info_list) {

  uint32_t admin_index = 0;
  auto existing_admin = admin_info_map.find(admin_info);

  // If admin was not processed yet
  if (existing_admin == admin_info_map.end()) {

    // Assign new admin index
    admin_index = admin_info_list.size();

    // Add admin info to list
    admin_info_list.emplace_back(admin_info);

    // Add admin info/index pair to map
    admin_info_map.emplace(admin_info, admin_index);
  } // Use known admin
  else {
    admin_index = existing_admin->second;
  }
  return admin_index;
}

void AssignAdmins(const AttributesController& controller,
                  TripLeg& trip_path,
                  const std::vector<AdminInfo>& admin_info_list) {
  if (controller.category_attribute_enabled(kAdminCategory)) {
    // Assign the admins
    trip_path.mutable_admin()->Reserve(admin_info_list.size());
    for (const auto& admin_info : admin_info_list) {
      TripLeg_Admin* trip_admin = trip_path.add_admin();

      // Set country code if requested
      if (controller(kAdminCountryCode)) {
        trip_admin->set_country_code(admin_info.country_iso());
      }

      // Set country text if requested
      if (controller(kAdminCountryText)) {
        trip_admin->set_country_text(admin_info.country_text());
      }

      // Set state code if requested
      if (controller(kAdminStateCode)) {
        trip_admin->set_state_code(admin_info.state_iso());
      }

      // Set state text if requested
      if (controller(kAdminStateText)) {
        trip_admin->set_state_text(admin_info.state_text());
      }
    }
  }
}

// Helper function to get the iso country code from an edge using its endnode
inline std::string country_code_from_edge(const graph_tile_ptr& tile,
                                          const valhalla::baldr::DirectedEdge& de) {
  if (!tile) {
    return std::string();
  }
  return tile->admininfo(tile->node(de.endnode())->admin_index()).country_iso();
}

/**
 * Used to add or update incidents attached to the provided leg. We could do something more exotic to
 * avoid linear scan, like keeping a separate lookup outside of the pbf
 * @param leg        the leg to update
 * @param incident   the incident that applies
 * @param index      what shape index of the leg the index apples to
 */
void UpdateIncident(const std::shared_ptr<const valhalla::IncidentsTile>& incidents_tile,
                    TripLeg& leg,
                    const valhalla::IncidentsTile::Location* incident_location,
                    uint32_t index,
                    const graph_tile_ptr& end_node_tile,
                    const valhalla::baldr::DirectedEdge& de) {
  const uint64_t current_incident_id =
      valhalla::baldr::getIncidentMetadata(incidents_tile, *incident_location).id();
  auto found = std::find_if(leg.mutable_incidents()->begin(), leg.mutable_incidents()->end(),
                            [current_incident_id](const TripLeg::Incident& candidate) {
                              return current_incident_id == candidate.metadata().id();
                            });
  // Are we continuing an incident (this could be a hash look up)
  if (found != leg.mutable_incidents()->end()) {
    found->set_end_shape_index(index);
  } // We are starting a new incident
  else {
    auto* new_incident = leg.mutable_incidents()->Add();

    // Get the full incident metadata from the incident-tile
    const auto& meta = valhalla::baldr::getIncidentMetadata(incidents_tile, *incident_location);
    *new_incident->mutable_metadata() = meta;

    // Set iso country code (2 & 3 char codes) on the new incident obj created for this leg
    std::string country_code_iso_2 = country_code_from_edge(end_node_tile, de);
    if (!country_code_iso_2.empty()) {
      new_incident->mutable_metadata()->set_iso_3166_1_alpha2(country_code_iso_2.c_str());
    }
    std::string country_code_iso_3 = valhalla::baldr::get_iso_3166_1_alpha3(country_code_iso_2);
    if (!country_code_iso_3.empty()) {
      new_incident->mutable_metadata()->set_iso_3166_1_alpha3(country_code_iso_3.c_str());
    }

    new_incident->set_begin_shape_index(index);
    new_incident->set_end_shape_index(index);
  }
}

valhalla::TripLeg_Closure* fetch_last_closure_annotation(TripLeg& leg) {
  return leg.closures_size() ? leg.mutable_closures(leg.closures_size() - 1) : nullptr;
}

valhalla::TripLeg_Closure* fetch_or_create_closure_annotation(TripLeg& leg) {
  valhalla::TripLeg_Closure* closure = fetch_last_closure_annotation(leg);
  // If last closure annotation has its end index populated, create a new
  // closure annotation
  return (!closure || closure->has_end_shape_index_case()) ? leg.add_closures() : closure;
}

/**
 * Chops up the shape for an edge so that we have shape points where speeds change along the edge
 * and where incidents occur along the edge. Also sets the various per shape point attributes
 * such as time, distance, speed. Also updates the incidents list on the edge with their shape indices
 * @param controller
 * @param tile
 * @param edge
 * @param shape
 * @param shape_begin
 * @param leg
 * @param src_pct
 * @param tgt_pct
 * @param edge_seconds
 * @param cut_for_traffic
 * @param incidents
 */
void SetShapeAttributes(const AttributesController& controller,
                        const graph_tile_ptr& tile,
                        const graph_tile_ptr& end_node_tile,
                        const DirectedEdge* edge,
                        std::vector<PointLL>& shape,
                        size_t shape_begin,
                        TripLeg& leg,
                        double src_pct,
                        double tgt_pct,
                        double edge_seconds,
                        bool cut_for_traffic,
                        const valhalla::baldr::IncidentResult& incidents) {
  // TODO: if this is a transit edge then the costing will throw

  // bail if nothing to do
  if (!cut_for_traffic && incidents.start_index == incidents.end_index &&
      !controller.category_attribute_enabled(kShapeAttributesCategory)) {
    return;
  }

  // initialize shape_attributes once
  if (!leg.has_shape_attributes() &&
      controller.category_attribute_enabled(kShapeAttributesCategory)) {
    leg.mutable_shape_attributes();
  }

  // convenient for bundling info about the spots where we cut the shape
  struct cut_t {
    double percent_along;
    double speed; // meters per second
    uint8_t congestion;
    std::vector<const valhalla::IncidentsTile::Location*> incidents;
    bool closed;
  };

  // A list of percent along the edge, corresponding speed (meters per second), incident id
  std::vector<cut_t> cuts;
  double speed = (edge->length() * (tgt_pct - src_pct)) / edge_seconds;
  if (cut_for_traffic) {
    // TODO: we'd like to use the speed from traffic here but because there are synchronization
    // problems with those records changing between when we used them to make the path and when we
    // try to grab them again here, we instead rely on the total time from PathInfo and just do the
    // cutting for now
    const auto& traffic_speed = tile->trafficspeed(edge);
    if (traffic_speed.breakpoint1 > 0) {
      cuts.emplace_back(cut_t{traffic_speed.breakpoint1 / 255.0,
                              speed,
                              static_cast<std::uint8_t>(traffic_speed.congestion1),
                              {},
                              traffic_speed.closed(0)});
      if (traffic_speed.breakpoint2 > 0) {
        cuts.emplace_back(cut_t{traffic_speed.breakpoint2 / 255.0,
                                speed,
                                static_cast<std::uint8_t>(traffic_speed.congestion2),
                                {},
                                traffic_speed.closed(1)});
        if (traffic_speed.breakpoint2 < 255) {
          cuts.emplace_back(cut_t{1,
                                  speed,
                                  static_cast<std::uint8_t>(traffic_speed.congestion3),
                                  {},
                                  traffic_speed.closed(2)});
        }
      }
    }
  }

  // Cap the end so that we always have something to use
  if (cuts.empty() || cuts.back().percent_along < tgt_pct) {
    cuts.emplace_back(cut_t{tgt_pct, speed, UNKNOWN_CONGESTION_VAL, {}, false});
  }

  // sort the start and ends of the incidents along this edge
  for (auto incident_location_index = incidents.start_index;
       incident_location_index != incidents.end_index; ++incident_location_index) {
    if (incident_location_index >= incidents.tile->locations_size()) {
      throw std::logic_error(
          "invalid incident_location_index: " + std::to_string(incident_location_index) + " vs " +
          std::to_string(incidents.tile->locations_size()));
    }
    const auto& incident = incidents.tile->locations(incident_location_index);
    // if the incident is actually on the part of the edge we are using
    if (incident.start_offset() > tgt_pct || incident.end_offset() < src_pct)
      continue;
    // insert the start point and end points
    for (auto offset : {
             std::max((double)incident.start_offset(), src_pct),
             std::min((double)incident.end_offset(), tgt_pct),
         }) {
      // if this is clipped at the beginning of the edge then its not a new cut but we still need to
      // attach the incidents information to the leg
      if (offset == src_pct) {
        UpdateIncident(incidents.tile, leg, &incident, shape_begin, end_node_tile, *edge);
        continue;
      }

      // where does it go in the sorted list
      auto itr = std::partition_point(cuts.begin(), cuts.end(),
                                      [offset](const cut_t& c) { return c.percent_along < offset; });
      // there is already a cut here so we just add the incident
      if (itr != cuts.end() && itr->percent_along == offset) {
        itr->incidents.push_back(&incident);
      } // there wasnt a cut here so we need to make one
      else {
        cuts.insert(itr, cut_t{offset,
                               itr == cuts.end() ? speed : itr->speed,
                               itr == cuts.end() ? UNKNOWN_CONGESTION_VAL : itr->congestion,
                               {&incident},
                               itr == cuts.end() ? false : itr->closed});
      }
    }
  }

  // Find the first cut to the right of where we start on this edge
  auto edgeinfo = tile->edgeinfo(edge);
  double distance_total_pct = src_pct;
  auto cut_itr = std::find_if(cuts.cbegin(), cuts.cend(),
                              [distance_total_pct](const decltype(cuts)::value_type& s) {
                                return distance_total_pct <= s.percent_along;
                              });
  assert(cut_itr != cuts.cend());

  // reservations
  if (controller(kShapeAttributesTime)) {
    leg.mutable_shape_attributes()->mutable_time()->Reserve(leg.shape_attributes().time_size() +
                                                            shape.size() + cuts.size());
  }
  if (controller(kShapeAttributesLength)) {
    leg.mutable_shape_attributes()->mutable_length()->Reserve(leg.shape_attributes().length_size() +
                                                              shape.size() + cuts.size());
  }
  if (controller(kShapeAttributesSpeed)) {
    leg.mutable_shape_attributes()->mutable_speed()->Reserve(leg.shape_attributes().speed_size() +
                                                             shape.size() + cuts.size());
  }
  if (controller(kShapeAttributesSpeedLimit)) {
    leg.mutable_shape_attributes()->mutable_speed_limit()->Reserve(
        leg.shape_attributes().speed_limit_size() + shape.size() + cuts.size());
  }

  // Set the shape attributes
  for (auto i = shape_begin + 1; i < shape.size(); ++i) {
    // when speed changes we need to make a new shape point and continue from there
    double distance = shape[i].Distance(shape[i - 1]); // meters
    double distance_pct = distance / edge->length();
    double next_total = distance_total_pct + distance_pct;
    size_t shift = 0;
    if (next_total > cut_itr->percent_along && std::next(cut_itr) != cuts.cend()) {
      // Calculate where the cut point should be between these two existing shape points
      auto coef = (cut_itr->percent_along - distance_total_pct) / (next_total - distance_total_pct);
      auto point = shape[i - 1].PointAlongSegment(shape[i], coef);
      shape.insert(shape.begin() + i, point);
      next_total = cut_itr->percent_along;
      distance *= coef;
      shift = 1;
    }
    if (controller(kShapeAttributesClosure)) {
      // Process closure annotations
      if (cut_itr->closed) {
        // Found a closure. Fetch a new annotation, or the last closure
        // annotation if it does not have an end index set (meaning the shape
        // is still within an existing closure)
        ::valhalla::TripLeg_Closure* closure = fetch_or_create_closure_annotation(leg);
        if (!closure->has_begin_shape_index_case()) {
          closure->set_begin_shape_index(i - 1);
        }
      } else {
        // Not a closure, check if we need to set the end of an existing
        // closure annotation or not
        ::valhalla::TripLeg_Closure* closure = fetch_last_closure_annotation(leg);
        if (closure && !closure->has_end_shape_index_case()) {
          closure->set_end_shape_index(i - 1);
        }
      }
    }
    distance_total_pct = next_total;
    double time = distance / cut_itr->speed; // seconds
    if (std::isnan(time)) {
      time = 0.;
    }

    // Set shape attributes time per shape point if requested
    if (controller(kShapeAttributesTime)) {
      // convert time to milliseconds and then round to an integer
      leg.mutable_shape_attributes()->add_time((time * kMillisecondPerSec) + 0.5);
    }

    // Set shape attributes length per shape point if requested
    if (controller(kShapeAttributesLength)) {
      // convert length to decimeters and then round to an integer
      leg.mutable_shape_attributes()->add_length((distance * kDecimeterPerMeter) + 0.5);
    }

    // Set shape attributes speed per shape point if requested
    if (controller(kShapeAttributesSpeed)) {
      // convert speed to decimeters per sec and then round to an integer
      double decimeters_sec = (distance * kDecimeterPerMeter / time) + 0.5;
      if (std::isnan(decimeters_sec) || time == 0.) { // avoid NaN
        decimeters_sec = 0.;
      }
      leg.mutable_shape_attributes()->add_speed(decimeters_sec);
    }

    // Set the maxspeed if requested
    if (controller(kShapeAttributesSpeedLimit)) {
      leg.mutable_shape_attributes()->add_speed_limit(edgeinfo.speed_limit());
    }

    // Set the incidents if we just cut or we are at the end
    if ((shift || i == shape.size() - 1) && !cut_itr->incidents.empty()) {
      for (const auto* incident : cut_itr->incidents) {
        UpdateIncident(incidents.tile, leg, incident, i, end_node_tile, *edge);
      }
    }

    // If we just cut the shape we need to go on to the next marker only after setting the attribs
    std::advance(cut_itr, shift);
  }
}

// Set the bounding box (min,max lat,lon) for the shape
void SetBoundingBox(TripLeg& trip_path, std::vector<PointLL>& shape) {
  AABB2<PointLL> bbox(shape);
  LatLng* min_ll = trip_path.mutable_bbox()->mutable_min_ll();
  min_ll->set_lat(bbox.miny());
  min_ll->set_lng(bbox.minx());
  LatLng* max_ll = trip_path.mutable_bbox()->mutable_max_ll();
  max_ll->set_lat(bbox.maxy());
  max_ll->set_lng(bbox.maxx());
}

/**
 * Removes all edges but the one with the id that we are passing
 * @param location  The location
 * @param edge_id   The edge id to keep
 */
void RemovePathEdges(valhalla::Location* location, const GraphId& edge_id) {
  auto pos =
      std::find_if(location->correlation().edges().begin(), location->correlation().edges().end(),
                   [&edge_id](const valhalla::PathEdge& e) { return e.graph_id() == edge_id; });
  if (pos == location->correlation().edges().end())
    throw std::logic_error("Could not find matching edge candidate");

  if (location->correlation().edges_size() > 1) {
    location->mutable_correlation()
        ->mutable_edges()
        ->SwapElements(0, pos - location->correlation().edges().begin());
    location->mutable_correlation()
        ->mutable_edges()
        ->DeleteSubrange(1, location->correlation().edges_size() - 1);
  }
}

/**
 * Copy the subset of options::location into the tripleg::locations and remove edge candidates
 * that werent removed during the construction of the route
 */
void CopyLocations(TripLeg& trip_path,
                   const valhalla::Location& origin,
                   const std::vector<valhalla::Location>& intermediates,
                   const valhalla::Location& dest,
                   const std::vector<PathInfo>::const_iterator path_begin,
                   const std::vector<PathInfo>::const_iterator path_end) {
  // origin
  trip_path.add_location()->CopyFrom(origin);
  RemovePathEdges(&*trip_path.mutable_location()->rbegin(), path_begin->edgeid);
  // intermediates
  std::optional<uint32_t> last_shape_index = std::nullopt;
  for (const auto& intermediate : intermediates) {
    valhalla::Location* tp_intermediate = trip_path.add_location();
    tp_intermediate->CopyFrom(intermediate);
    // we can grab the right edge index in the path because we temporarily set it for trimming
    if (last_shape_index && intermediate.correlation().leg_shape_index() <= *last_shape_index) {
      throw std::logic_error("leg_shape_index not set for intermediate location");
    }
    last_shape_index = intermediate.correlation().leg_shape_index();
    RemovePathEdges(&*trip_path.mutable_location()->rbegin(),
                    (path_begin + *last_shape_index)->edgeid);
  }
  // destination
  trip_path.add_location()->CopyFrom(dest);
  RemovePathEdges(&*trip_path.mutable_location()->rbegin(), std::prev(path_end)->edgeid);
}

/**
 * Set begin and end heading if requested.
 * @param  trip_edge  Trip path edge to add headings.
 * @param  controller Controller specifying attributes to add to trip edge.
 * @param  edge       Directed edge.
 * @param  shape      Trip shape.
 */
void SetHeadings(TripLeg_Edge* trip_edge,
                 const AttributesController& controller,
                 const DirectedEdge* edge,
                 const std::vector<PointLL>& shape,
                 const uint32_t begin_index) {
  if (controller(kEdgeBeginHeading) || controller(kEdgeEndHeading)) {
    float offset = GetOffsetForHeading(edge->classification(), edge->use());
    if (controller(kEdgeBeginHeading)) {
      trip_edge->set_begin_heading(
          std::round(PointLL::HeadingAlongPolyline(shape, offset, begin_index, shape.size() - 1)));
    }
    if (controller(kEdgeEndHeading)) {
      trip_edge->set_end_heading(
          std::round(PointLL::HeadingAtEndOfPolyline(shape, offset, begin_index, shape.size() - 1)));
    }
  }
}

/**
 * Add landmarks in the directed edge to trip edge.
 * @param  edgeinfo    Edge info of the directed edge.
 * @param  trip_edge   Trip path edge to add landmarks.
 * @param  controller  Controller specifying whether we want landmarks in the graph to come out the
 * other side.
 * @param  edge        Directed edge where the landmarks are stored.
 * @param  shape       Trip shape.
 */
void AddLandmarks(const EdgeInfo& edgeinfo,
                  TripLeg_Edge* trip_edge,
                  const AttributesController& controller,
                  const DirectedEdge* edge,
                  const std::vector<PointLL>& shape,
                  const uint32_t begin_index) {
  if (!controller(kEdgeLandmarks)) {
    return;
  }

  const auto landmark_range = edgeinfo.GetTags().equal_range(baldr::TaggedValue::kLandmark);
  for (auto it = landmark_range.first; it != landmark_range.second; ++it) {
    Landmark lan(it->second);
    PointLL landmark_point = {lan.lng, lan.lat};

    // find the closed point on edge to the landmark
    auto closest = landmark_point.ClosestPoint(shape, begin_index);
    // TODO: in the future maybe we could allow a request option to have a tighter threshold on
    // how far landmarks should be away from an edge

    // add the landmark to trip leg
    auto* landmark = trip_edge->mutable_landmarks()->Add();
    landmark->set_name(lan.name);
    landmark->set_type(static_cast<valhalla::RouteLandmark::Type>(lan.type));
    landmark->mutable_lat_lng()->set_lng(lan.lng);
    landmark->mutable_lat_lng()->set_lat(lan.lat);

    // calculate the landmark's distance along the edge
    // that is to accumulate distance from the begin point to the closest point to it on the edge
    int closest_idx = std::get<2>(closest);
    double distance_along_edge = 0;
    for (int idx = begin_index + 1; idx <= closest_idx; ++idx) {
      distance_along_edge += shape[idx].Distance(shape[idx - 1]);
    }
    distance_along_edge += shape[closest_idx].Distance(std::get<0>(closest));
    // the overall distance shouldn't be larger than edge length
    distance_along_edge = std::min(distance_along_edge, static_cast<double>(edge->length()));
    landmark->set_distance(distance_along_edge);
    // check which side of the edge the landmark is on
    // quirks of the ClosestPoint function
    bool is_right = closest_idx == (int)shape.size() - 1
                        ? landmark_point.IsLeft(shape[closest_idx - 1], shape[closest_idx]) < 0
                        : landmark_point.IsLeft(shape[closest_idx], shape[closest_idx + 1]) < 0;
    landmark->set_right(is_right);
  }
}

// Populate the specified sign element with the specified sign attributes including pronunciation
// attributes if they exist
void PopulateSignElement(uint32_t sign_index,
                         const SignInfo& sign,
                         const LinguisticMap& linguistics,
                         valhalla::TripSignElement* sign_element) {
  sign_element->set_text(sign.text());
  sign_element->set_is_route_number(sign.is_route_num());

  // Assign pronunciation alphabet and value if they exist
  const auto iter = linguistics.find(sign_index);
  if (iter != linguistics.end()) {

    // Lang saved with pronunciation
    auto lang = static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second));
    if (lang != Language::kNone) {
      sign_element->set_language_tag(GetTripLanguageTag(lang));
    }

    auto alphabet = static_cast<valhalla::baldr::PronunciationAlphabet>(
        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second));

    if (alphabet != PronunciationAlphabet::kNone) {

      auto* pronunciation = sign_element->mutable_pronunciation();
      pronunciation->set_alphabet(GetTripPronunciationAlphabet(alphabet));
      pronunciation->set_value(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second));
    }
  }
}

// Walk the edge_signs, add sign information onto the trip_sign, honoring which to
// add per the attributes-controller.
void AddSignInfo(const AttributesController& controller,
                 const std::vector<SignInfo>& edge_signs,
                 const LinguisticMap& linguistics,
                 valhalla::TripSign* trip_sign) {

  if (!edge_signs.empty()) {
    uint32_t sign_index = 0;
    for (const auto& sign : edge_signs) {
      switch (sign.type()) {
        case valhalla::baldr::Sign::Type::kExitNumber: {
          if (controller.attributes.at(kEdgeSignExitNumber)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_exit_numbers()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kExitBranch: {
          if (controller.attributes.at(kEdgeSignExitBranch)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_exit_onto_streets()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kExitToward: {
          if (controller.attributes.at(kEdgeSignExitToward)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_exit_toward_locations()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kExitName: {
          if (controller.attributes.at(kEdgeSignExitName)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_exit_names()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kGuideBranch: {
          if (controller.attributes.at(kEdgeSignGuideBranch)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_guide_onto_streets()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kGuideToward: {
          if (controller.attributes.at(kEdgeSignGuideToward)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_guide_toward_locations()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kGuidanceViewJunction: {
          if (controller.attributes.at(kEdgeSignGuidanceViewJunction)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_guidance_view_junctions()->Add());
          }
          break;
        }
        case valhalla::baldr::Sign::Type::kGuidanceViewSignboard: {
          if (controller.attributes.at(kEdgeSignGuidanceViewSignboard)) {
            PopulateSignElement(sign_index, sign, linguistics,
                                trip_sign->mutable_guidance_view_signboards()->Add());
          }
          break;
        }
        default: {
          break;
        }
      }
      ++sign_index;
    }
  }
}

void FilterUnneededStreetNumbers(
    std::vector<std::tuple<std::string, bool, uint8_t>>& names_and_types) {
  if (names_and_types.size() < 2) {
    return;
  }
  auto it = names_and_types.begin();
  while (it != names_and_types.end()) {
    if (std::get<1>(*it) == true && names_and_types.size() > 1) {
      it = names_and_types.erase(it);
    } else {
      it++;
    }
  }
}

/**
 * Set elevation along the edge if requested.
 * @param  trip_edge   Trip path edge to add elevation.
 * @param  start_pct   Start percent
 * @param  end_pct     End percent
 * @param  start_node  Nodeinfo for start of the edge.
 * @param  edge        Directed edge
 * @param  tile        Graph tile of the edge.
 * @param  graphreader Graphreader in case end node is in a different tile.
 */
void SetElevation(TripLeg_Edge* trip_edge,
                  const double start_pct,
                  const double end_pct,
                  const NodeInfo* start_node,
                  const DirectedEdge* edge,
                  const graph_tile_ptr& tile,
                  GraphReader& graphreader) {

  // Lambda to get elevation at specified distance
  double interval = 0.0;
  const auto find_elevation = [&interval](const std::vector<float> elevation, const double d) {
    // Find index based on the stored interval and the desired distance
    uint32_t index = static_cast<uint32_t>(d / interval);
    if (index >= elevation.size() - 1) {
      return elevation.back();
    }

    // Interpolate between this vertex and the next
    double pct = (d / interval) - index;
    return static_cast<float>((elevation[index] * (1.0 - pct)) + (elevation[index + 1] * pct));
  };

  // Add the elevation at the start node to the elevation vector
  float h1 = start_node->elevation();

  // Get encoded elevation from EdgeInfo edge
  auto encoded = tile->edgeinfo(edge).encoded_elevation(edge->length(), interval);

  // Get the end node and its elevation (if end tile is null just set elevation at
  // the end node to be same as at start node - this should be rare)
  float h2 = h1;
  auto end_tile = graphreader.GetGraphTile(edge->endnode());
  if (end_tile != nullptr) {
    h2 = end_tile->node(edge->endnode())->elevation();
  }

  // Decode elevation and reverse if edge is not forward direction
  std::vector<float> elevation = decode_elevation(encoded, h1, h2, edge->forward());

  // Add to trip edge (protect against both start and end percent == 0.0)
  if (((0.0 < start_pct && start_pct < 1.0) || (0.0 < end_pct && end_pct < 1.0)) &&
      start_pct != end_pct) {
    // Trim elevation - find a new sampling interval based on the partial length
    double partial_length = static_cast<double>(edge->length()) * (end_pct - start_pct);
    double new_interval = sampling_interval(partial_length);
    trip_edge->set_elevation_sampling_interval(new_interval);

    // Add elevation along this portion of the edge
    double d = edge->length() * start_pct;
    double end_distance = edge->length() * end_pct;
    while (d < end_distance + kEpsilon) {
      trip_edge->mutable_elevation()->Add(find_elevation(elevation, d));
      d += new_interval;
    }

    // Validate new size
    if (trip_edge->elevation_size() - 1 != static_cast<int32_t>(partial_length / new_interval)) {
      LOG_ERROR("TRIMMED elevation is wrong size");
    }
  } else {
    // Set trip_edge sampling interval and elevation vector
    trip_edge->set_elevation_sampling_interval(interval);
    for (auto e : elevation) {
      trip_edge->mutable_elevation()->Add(e);
    }
  }
}

void ProcessNonTaggedValue(valhalla::StreetName* trip_edge_name,
                           const LinguisticMap& linguistics,
                           const std::tuple<std::string, bool, uint8_t>& name_and_type,
                           const uint8_t name_index) {
  // Assign name and type
  trip_edge_name->set_value(std::get<0>(name_and_type));
  trip_edge_name->set_is_route_number(std::get<1>(name_and_type));

  const auto iter = linguistics.find(name_index);

  if (iter != linguistics.end()) {

    auto lang = static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second));
    if (lang != Language::kNone) {
      trip_edge_name->set_language_tag(GetTripLanguageTag(lang));
    }

    auto alphabet = static_cast<valhalla::baldr::PronunciationAlphabet>(
        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second));

    if (alphabet != PronunciationAlphabet::kNone) {

      auto* pronunciation = trip_edge_name->mutable_pronunciation();
      pronunciation->set_alphabet(
          GetTripPronunciationAlphabet(static_cast<valhalla::baldr::PronunciationAlphabet>(
              std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second))));
      pronunciation->set_value(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second));
    }
  }
}

/**
 * Add trip intersecting edge.
 * @param  controller   Controller to determine which attributes to set.
 * @param  directededge Directed edge on the path.
 * @param  prev_de  Previous directed edge on the path.
 * @param  local_edge_index  Index of the local intersecting path edge at intersection.
 * @param  nodeinfo  Node information of the intersection.
 * @param  trip_node  Trip node that will store the intersecting edge information.
 * @param  intersecting_de Intersecting directed edge. Will be nullptr except when
 *                         on the local hierarchy.
 */
void AddTripIntersectingEdge(const AttributesController& controller,
                             const graph_tile_ptr& graphtile,
                             const DirectedEdge* directededge,
                             const DirectedEdge* prev_de,
                             uint32_t local_edge_index,
                             const NodeInfo* nodeinfo,
                             TripLeg_Node* trip_node,
                             const DirectedEdge* intersecting_de,
                             bool blind_instructions) {
  TripLeg_IntersectingEdge* intersecting_edge = trip_node->add_intersecting_edge();

  // Set the heading for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeBeginHeading)) {
    intersecting_edge->set_begin_heading(nodeinfo->heading(local_edge_index));
  }

  Traversability traversability = Traversability::kNone;
  // Determine walkability
  if (intersecting_de->forwardaccess() & kPedestrianAccess) {
    traversability = (intersecting_de->reverseaccess() & kPedestrianAccess)
                         ? Traversability::kBoth
                         : Traversability::kForward;
  } else {
    traversability = (intersecting_de->reverseaccess() & kPedestrianAccess)
                         ? Traversability::kBackward
                         : Traversability::kNone;
  }
  // Set the walkability flag for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeWalkability)) {
    intersecting_edge->set_walkability(GetTripLegTraversability(traversability));
  }

  traversability = Traversability::kNone;
  // Determine cyclability
  if (intersecting_de->forwardaccess() & kBicycleAccess) {
    traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ? Traversability::kBoth
                                                                         : Traversability::kForward;
  } else {
    traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ? Traversability::kBackward
                                                                         : Traversability::kNone;
  }
  // Set the cyclability flag for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeCyclability)) {
    intersecting_edge->set_cyclability(GetTripLegTraversability(traversability));
  }

  // Set the driveability flag for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeDriveability)) {
    intersecting_edge->set_driveability(
        GetTripLegTraversability(nodeinfo->local_driveability(local_edge_index)));
  }

  // Set the previous/intersecting edge name consistency if requested
  if (controller(kNodeIntersectingEdgeFromEdgeNameConsistency)) {
    bool name_consistency =
        (prev_de == nullptr) ? false : prev_de->name_consistency(local_edge_index);
    intersecting_edge->set_prev_name_consistency(name_consistency);
  }

  // Set the current/intersecting edge name consistency if requested
  if (controller(kNodeIntersectingEdgeToEdgeNameConsistency)) {
    intersecting_edge->set_curr_name_consistency(directededge->name_consistency(local_edge_index));
  }

  // Add names to edge if requested
  if (controller.attributes.at(kEdgeNames)) {

    auto edgeinfo = graphtile->edgeinfo(intersecting_de);
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    if (blind_instructions) {
      FilterUnneededStreetNumbers(names_and_types);
    }
    intersecting_edge->mutable_name()->Reserve(names_and_types.size());
    const auto linguistics = edgeinfo.GetLinguisticMap();

    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      switch (std::get<2>(name_and_type)) {
        case kNotTagged: {
          ProcessNonTaggedValue(intersecting_edge->mutable_name()->Add(), linguistics, name_and_type,
                                name_index);
          break;
        }
        default:
          // Skip the rest tagged names
          LOG_TRACE(std::string("skipped tagged value= ") +
                    std::to_string(std::get<2>(name_and_type)));
          break;
      }
      ++name_index;
    }
  }

  // Set the use for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeUse)) {
    intersecting_edge->set_use(GetTripLegUse(intersecting_de->use()));
  }

  // Set the road class for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeRoadClass)) {
    intersecting_edge->set_road_class(GetRoadClass(intersecting_de->classification()));
  }

  // Set the lane count for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeLaneCount)) {
    intersecting_edge->set_lane_count(intersecting_de->lanecount());
  }

  // Set the sign info for the intersecting edge if requested
  if (controller(kNodeIntersectingEdgeSignInfo)) {
    if (intersecting_de->sign()) {
      LinguisticMap linguistics;
      std::vector<SignInfo> edge_signs =
          graphtile->GetSigns(intersecting_de - graphtile->directededge(0), linguistics);
      if (!edge_signs.empty()) {
        valhalla::TripSign* sign = intersecting_edge->mutable_sign();
        AddSignInfo(controller, edge_signs, linguistics, sign);
      }
    }
  }
}

/**
 * Adds the intersecting edges in the graph at the current node. Skips edges which are on the path
 * as well as those which are duplicates due to shortcut edges.
 * @param controller               tells us what info we should add about the intersecting edges
 * @param start_tile               the tile which contains the node
 * @param node                     the node at which we are copying intersecting edges
 * @param directededge             the current edge leaving the current node in the path
 * @param prev_de                  the previous edge in the path
 * @param prior_opp_local_index    opposing edge local index of previous edge in the path
 * @param graphreader              graph reader for graph access
 * @param trip_node                pbf node in the pbf structure we are building
 * @param blind_instructions       whether instructions for blind users are requested
 */
void AddIntersectingEdges(const AttributesController& controller,
                          const graph_tile_ptr& start_tile,
                          const NodeInfo* node,
                          const DirectedEdge* directededge,
                          const DirectedEdge* prev_de,
                          uint32_t prior_opp_local_index,
                          GraphReader& graphreader,
                          valhalla::TripLeg::Node* trip_node,
                          const bool blind_instructions) {
  /* Add connected edges from the start node. Do this after the first trip
     edge is added

     Our path is from 1 to 2 to 3 (nodes) to ... n nodes.
     Each letter represents the edge info.
     So at node 2, we will store the edge info for D and we will store the
     intersecting edge info for B, C, E, F, and G.  We need to make sure
     that we don't store the edge info from A and D again.

         (X)    (3)   (X)
           \\   ||   //
          C \\ D|| E//
             \\ || //
          B   \\||//   F
     (X)======= (2) ======(X)
                ||\\
              A || \\ G
                ||  \\
                (1)  (X)
  */

  // prepare for some edges
  trip_node->mutable_intersecting_edge()->Reserve(node->local_edge_count());

  // Iterate through edges on this level to find any intersecting edges
  // Follow any upwards or downward transitions
  const DirectedEdge* intersecting_edge = start_tile->directededge(node->edge_index());
  for (uint32_t idx1 = 0; idx1 < node->edge_count(); ++idx1, intersecting_edge++) {

    // Skip shortcut edges AND the opposing edge of the previous edge in the path AND
    // the current edge in the path AND the superseded edge of the current edge in the path
    // if the current edge in the path is a shortcut
    if (intersecting_edge->is_shortcut() ||
        intersecting_edge->localedgeidx() == prior_opp_local_index ||
        intersecting_edge->localedgeidx() == directededge->localedgeidx() ||
        (directededge->is_shortcut() && directededge->shortcut() & intersecting_edge->superseded()) ||
        intersecting_edge->use() == Use::kConstruction) {
      continue;
    }

    // Add intersecting edges on the same hierarchy level and not on the path
    AddTripIntersectingEdge(controller, start_tile, directededge, prev_de,
                            intersecting_edge->localedgeidx(), node, trip_node, intersecting_edge,
                            blind_instructions);
  }

  // Add intersecting edges on different levels (follow NodeTransitions)
  if (node->transition_count() > 0) {
    const NodeTransition* trans = start_tile->transition(node->transition_index());
    for (uint32_t i = 0; i < node->transition_count(); ++i, ++trans) {
      // Get the end node tile and its directed edges
      GraphId endnode = trans->endnode();
      graph_tile_ptr endtile = graphreader.GetGraphTile(endnode);
      if (endtile == nullptr) {
        continue;
      }
      const NodeInfo* nodeinfo2 = endtile->node(endnode);
      const DirectedEdge* intersecting_edge2 = endtile->directededge(nodeinfo2->edge_index());
      for (uint32_t idx2 = 0; idx2 < nodeinfo2->edge_count(); ++idx2, intersecting_edge2++) {
        // Skip shortcut edges and edges on the path
        if (intersecting_edge2->is_shortcut() ||
            intersecting_edge2->localedgeidx() == prior_opp_local_index ||
            intersecting_edge2->localedgeidx() == directededge->localedgeidx() ||
            intersecting_edge2->use() == Use::kConstruction) {
          continue;
        }

        AddTripIntersectingEdge(controller, endtile, directededge, prev_de,
                                intersecting_edge2->localedgeidx(), nodeinfo2, trip_node,
                                intersecting_edge2, blind_instructions);
      }
    }
  }
}

void ProcessTunnelBridgeTaggedValue(valhalla::StreetName* trip_edge_name,
                                    const LinguisticMap& linguistics,
                                    const std::tuple<std::string, bool, uint8_t>& name_and_type,
                                    const uint8_t name_index) {

  trip_edge_name->set_value(std::get<0>(name_and_type));
  trip_edge_name->set_is_route_number(std::get<1>(name_and_type));

  const auto iter = linguistics.find(name_index);
  if (iter != linguistics.end()) {

    auto lang = static_cast<Language>(std::get<kLinguisticMapTupleLanguageIndex>(iter->second));
    if (lang != Language::kNone) {
      trip_edge_name->set_language_tag(GetTripLanguageTag(lang));
    }

    auto alphabet = static_cast<valhalla::baldr::PronunciationAlphabet>(
        std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second));

    if (alphabet != PronunciationAlphabet::kNone) {

      auto* pronunciation = trip_edge_name->mutable_pronunciation();
      pronunciation->set_alphabet(
          GetTripPronunciationAlphabet(static_cast<valhalla::baldr::PronunciationAlphabet>(
              std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter->second))));
      pronunciation->set_value(std::get<kLinguisticMapTuplePronunciationIndex>(iter->second));
    }
  }
}

/**
 * Add trip edge. (TODO more comments)
 * @param  controller         Controller to determine which attributes to set.
 * @param  edge               Identifier of an edge within the tiled, hierarchical graph.
 * @param  trip_id            Trip Id (0 if not a transit edge).
 * @param  block_id           Transit block Id (0 if not a transit edge)
 * @param  mode               Travel mode for the edge: Biking, walking, etc.
 * @param  directededge       Directed edge information.
 * @param  drive_right        Right side driving for this edge.
 * @param  trip_node          Trip node to add the edge information to.
 * @param  graphtile          Graph tile for accessing data.
 * @param  second_of_week     The time, from the beginning of the week in seconds at which
 *                            the path entered this edge (always monday at noon on timeless route)
 * @param  start_node_idx     The start node index
 * @param  has_junction_name  True if named junction exists, false otherwise
 * @param  start_tile         The start tile of the start node
 * @param  blind_instructions Whether instructions should be generated for blind users
 *
 */
TripLeg_Edge* AddTripEdge(const AttributesController& controller,
                          const GraphId& edge,
                          const uint32_t trip_id,
                          const uint32_t block_id,
                          const sif::TravelMode mode,
                          const uint8_t travel_type,
                          const std::shared_ptr<sif::DynamicCost>& costing,
                          const DirectedEdge* directededge,
                          const bool drive_on_right,
                          TripLeg_Node* trip_node,
                          const graph_tile_ptr& graphtile,
                          const baldr::TimeInfo& time_info,
                          const uint32_t start_node_idx,
                          const bool has_junction_name,
                          const graph_tile_ptr& start_tile,
                          const uint8_t restrictions_idx,
                          float elapsed_secs,
                          bool blind_instructions) {

  // Index of the directed edge within the tile
  uint32_t idx = edge.id();
  TripLeg_Edge* trip_edge = trip_node->mutable_edge();

  // Get the edgeinfo
  auto edgeinfo = graphtile->edgeinfo(directededge);

  // Add names to edge if requested
  if (controller(kEdgeNames)) {
    auto names_and_types = edgeinfo.GetNamesAndTypes(true);
    if (blind_instructions)
      FilterUnneededStreetNumbers(names_and_types);
    trip_edge->mutable_name()->Reserve(names_and_types.size());
    const auto linguistics = edgeinfo.GetLinguisticMap();

    uint8_t name_index = 0;
    for (const auto& name_and_type : names_and_types) {
      switch (std::get<2>(name_and_type)) {
        case kNotTagged: {
          ProcessNonTaggedValue(trip_edge->mutable_name()->Add(), linguistics, name_and_type,
                                name_index);
          break;
        }
        case kTunnelTag:
        case kBridgeTag: {
          ProcessTunnelBridgeTaggedValue(trip_edge->mutable_tunnel_name()->Add(), linguistics,
                                         name_and_type, name_index);
          break;
        }
        default:
          // Skip the rest tagged names
          LOG_TRACE(std::string("skipped tagged value= ") +
                    std::to_string(std::get<2>(name_and_type)));
          break;
      }
      ++name_index;
    }
  }

  // Add tagged names to the edge if requested
  if (controller(kEdgeTaggedValues)) {
    const auto& tagged_values_and_types = edgeinfo.GetTags();
    trip_edge->mutable_tagged_value()->Reserve(tagged_values_and_types.size());
    for (const auto& tagged_value_and_type : tagged_values_and_types) {
      auto* trip_edge_tag_name = trip_edge->mutable_tagged_value()->Add();
      trip_edge_tag_name->set_value(tagged_value_and_type.second);
      trip_edge_tag_name->set_type(
          static_cast<TaggedValue_Type>(static_cast<uint8_t>(tagged_value_and_type.first)));
    }
  }

#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE(std::string("wayid=") + std::to_string(edgeinfo.wayid()));
#endif

  // Set the signs (if the directed edge has sign information) and if requested
  if (directededge->sign()) {
    // Add the edge signs
    LinguisticMap linguistics;
    std::vector<SignInfo> edge_signs = graphtile->GetSigns(idx, linguistics);
    if (!edge_signs.empty()) {
      valhalla::TripSign* sign = trip_edge->mutable_sign();
      AddSignInfo(controller, edge_signs, linguistics, sign);
    }
  }

  // Process the named junctions at nodes
  if (has_junction_name && start_tile) {
    // Add the node signs
    LinguisticMap linguistics;
    std::vector<SignInfo> node_signs = start_tile->GetSigns(start_node_idx, linguistics, true);
    if (!node_signs.empty()) {
      valhalla::TripSign* trip_sign = trip_edge->mutable_sign();
      uint32_t sign_index = 0;
      for (const auto& sign : node_signs) {
        switch (sign.type()) {
          case valhalla::baldr::Sign::Type::kJunctionName: {
            if (controller.attributes.at(kEdgeSignJunctionName)) {
              PopulateSignElement(sign_index, sign, linguistics,
                                  trip_sign->mutable_junction_names()->Add());
            }
            break;
          }
          default:
            break;
        }
        ++sign_index;
      }
    }
  }

  // If turn lanes exist
  if (directededge->turnlanes()) {
    auto turnlanes = graphtile->turnlanes(idx);
    trip_edge->mutable_turn_lanes()->Reserve(turnlanes.size());
    for (auto tl : turnlanes) {
      TurnLane* turn_lane = trip_edge->add_turn_lanes();
      turn_lane->set_directions_mask(tl);
    }
  }

  // Set road class if requested
  if (controller(kEdgeRoadClass)) {
    trip_edge->set_road_class(GetRoadClass(directededge->classification()));
  }

  // Set speed if requested
  // TODO: what to do about transit edges?
  if (controller(kEdgeSpeed)) {
    // TODO: could get better precision speed here by calling GraphTile::GetSpeed but we'd need to
    // know whether or not the costing actually cares about the speed of the edge. Perhaps a
    // refactor of costing to have a GetSpeed function which EdgeCost calls internally but which we
    // can also call externally
    double speed = 0;
    if (mode == sif::TravelMode::kPublicTransit) {
      // TODO(nils): get the actual speed here by passing in the elapsed seconds (or the whole
      // pathinfo)
      speed = directededge->length() / elapsed_secs * kMetersPerSectoKPH;
    } else {
      uint8_t flow_sources;
      speed = directededge->length() /
              costing->EdgeCost(directededge, graphtile, time_info, flow_sources).secs *
              kMetersPerSectoKPH;
    }
    trip_edge->set_speed(speed);
  }

  // Set country crossing if requested
  if (controller(kEdgeCountryCrossing)) {
    trip_edge->set_country_crossing(directededge->ctry_crossing());
  }

  // Set forward if requested
  if (controller(kEdgeForward)) {
    trip_edge->set_forward(directededge->forward());
  }

  uint8_t kAccess = 0;
  if (mode == sif::TravelMode::kBicycle) {
    kAccess = kBicycleAccess;
  } else if (mode == sif::TravelMode::kDrive) {
    kAccess = kAutoAccess;
  } else if (mode == sif::TravelMode::kPedestrian || mode == sif::TravelMode::kPublicTransit) {
    kAccess = kPedestrianAccess;
  }

  // Test whether edge is traversed forward or reverse
  if (directededge->forward()) {
    // Set traversability for forward directededge if requested
    if (controller(kEdgeTraversability)) {
      if ((directededge->forwardaccess() & kAccess) && (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBoth);
      } else if ((directededge->forwardaccess() & kAccess) &&
                 !(directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kForward);
      } else if (!(directededge->forwardaccess() & kAccess) &&
                 (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBackward);
      } else {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kNone);
      }
    }
  } else {
    // Set traversability for reverse directededge if requested
    if (controller(kEdgeTraversability)) {
      if ((directededge->forwardaccess() & kAccess) && (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBoth);
      } else if (!(directededge->forwardaccess() & kAccess) &&
                 (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kForward);
      } else if ((directededge->forwardaccess() & kAccess) &&
                 !(directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBackward);
      } else {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kNone);
      }
    }
  }

  if (directededge->access_restriction() && restrictions_idx != kInvalidRestriction) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        graphtile->GetAccessRestrictions(edge.id(), costing->access_mode());
    trip_edge->mutable_restriction()->set_type(
        static_cast<uint32_t>(restrictions[restrictions_idx].type()));
  }

  trip_edge->set_has_time_restrictions(restrictions_idx != kInvalidRestriction);

  // Set the trip path use based on directed edge use if requested
  if (controller(kEdgeUse)) {
    trip_edge->set_use(GetTripLegUse(directededge->use()));
  }

  // Set toll flag if requested
  if (directededge->toll() && controller(kEdgeToll)) {
    trip_edge->set_toll(true);
  }

  // Set unpaved flag if requested
  if (directededge->unpaved() && controller(kEdgeUnpaved)) {
    trip_edge->set_unpaved(true);
  }

  // Set tunnel flag if requested
  if (directededge->tunnel() && controller(kEdgeTunnel)) {
    trip_edge->set_tunnel(true);
  }

  // Set bridge flag if requested
  if (directededge->bridge() && controller(kEdgeBridge)) {
    trip_edge->set_bridge(true);
  }

  // Set roundabout flag if requested
  if (directededge->roundabout() && controller(kEdgeRoundabout)) {
    trip_edge->set_roundabout(true);
  }

  // Set internal intersection flag if requested
  if (directededge->internal() && controller(kEdgeInternalIntersection)) {
    trip_edge->set_internal_intersection(true);
  }

  // Set drive_on_right if requested
  if (controller(kEdgeDriveOnRight)) {
    trip_edge->set_drive_on_left(!drive_on_right);
  }

  // Set surface if requested
  if (controller(kEdgeSurface)) {
    trip_edge->set_surface(GetTripLegSurface(directededge->surface()));
  }

  if (directededge->destonly() && controller(kEdgeDestinationOnly)) {
    trip_edge->set_destination_only(directededge->destonly());
  }

  // Set indoor flag if requested
  if (directededge->indoor() && controller(kEdgeIndoor)) {
    trip_edge->set_indoor(true);
  }

  // Set the mode and travel type
  if (mode == sif::TravelMode::kBicycle) {
    // Override bicycle mode with pedestrian if dismount flag or steps
    if (directededge->dismount() || directededge->use() == Use::kSteps) {
      if (controller(kEdgeTravelMode)) {
        trip_edge->set_travel_mode(valhalla::TravelMode::kPedestrian);
      }
      if (controller(kEdgePedestrianType)) {
        trip_edge->set_pedestrian_type(valhalla::PedestrianType::kFoot);
      }
    } else {
      if (controller(kEdgeTravelMode)) {
        trip_edge->set_travel_mode(valhalla::TravelMode::kBicycle);
      }
      if (controller(kEdgeBicycleType)) {
        trip_edge->set_bicycle_type(GetTripLegBicycleType(travel_type));
      }
    }
  } else if (mode == sif::TravelMode::kDrive) {
    if (controller(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(valhalla::TravelMode::kDrive);
    }
    if (controller(kEdgeVehicleType)) {
      trip_edge->set_vehicle_type(GetTripLegVehicleType(travel_type));
    }
  } else if (mode == sif::TravelMode::kPedestrian) {
    if (controller(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(valhalla::TravelMode::kPedestrian);
    }
    if (controller(kEdgePedestrianType)) {
      trip_edge->set_pedestrian_type(GetTripLegPedestrianType(travel_type));
    }
  } else if (mode == sif::TravelMode::kPublicTransit) {
    if (controller(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(valhalla::TravelMode::kTransit);
    }
  }

  // Set edge id (graphid value) if requested
  if (controller(kEdgeId)) {
    trip_edge->set_id(edge.value);
  }

  // Set way id (base data id) if requested
  if (controller(kEdgeWayId)) {
    trip_edge->set_way_id(edgeinfo.wayid());
  }

  // Set weighted grade if requested
  if (controller(kEdgeWeightedGrade)) {
    trip_edge->set_weighted_grade((directededge->weighted_grade() - 6.f) / 0.6f);
  }

  // Set maximum upward and downward grade if requested (set to kNoElevationData if unavailable)
  if (controller(kEdgeMaxUpwardGrade)) {
    if (graphtile->header()->has_elevation()) {
      trip_edge->set_max_upward_grade(directededge->max_up_slope());
    } else {
      trip_edge->set_max_upward_grade(kNoElevationData);
    }
  }
  if (controller(kEdgeMaxDownwardGrade)) {
    if (graphtile->header()->has_elevation()) {
      trip_edge->set_max_downward_grade(directededge->max_down_slope());
    } else {
      trip_edge->set_max_downward_grade(kNoElevationData);
    }
  }

  // Set mean elevation if requested (will be kNoElevationData if unavailable)
  if (controller(kEdgeMeanElevation)) {
    trip_edge->set_mean_elevation(edgeinfo.mean_elevation());
  }

  if (controller(kEdgeLaneCount)) {
    trip_edge->set_lane_count(directededge->lanecount());
  }

  if (directededge->laneconnectivity() && controller(kEdgeLaneConnectivity)) {
    auto laneconnectivity = graphtile->GetLaneConnectivity(idx);
    trip_edge->mutable_lane_connectivity()->Reserve(laneconnectivity.size());
    for (const auto& l : laneconnectivity) {
      TripLeg_LaneConnectivity* path_lane = trip_edge->add_lane_connectivity();
      path_lane->set_from_way_id(l.from());
      path_lane->set_to_lanes(l.to_lanes());
      path_lane->set_from_lanes(l.from_lanes());
    }
  }

  if (directededge->cyclelane() != CycleLane::kNone && controller(kEdgeCycleLane)) {
    trip_edge->set_cycle_lane(GetTripLegCycleLane(directededge->cyclelane()));
  }

  if (controller(kEdgeBicycleNetwork)) {
    trip_edge->set_bicycle_network(directededge->bike_network());
  }

  if (controller(kEdgeSacScale)) {
    trip_edge->set_sac_scale(GetTripLegSacScale(directededge->sac_scale()));
  }

  if (controller(kEdgeShoulder)) {
    trip_edge->set_shoulder(directededge->shoulder());
  }

  if (controller(kEdgeSidewalk)) {
    if (directededge->sidewalk_left() && directededge->sidewalk_right()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kBothSides);
    } else if (directededge->sidewalk_left()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kLeft);
    } else if (directededge->sidewalk_right()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kRight);
    }
  }

  if (controller(kEdgeDensity)) {
    trip_edge->set_density(directededge->density());
  }

  if (controller(kEdgeIsUrban)) {
    bool is_urban = (directededge->density() > 8) ? true : false;
    trip_edge->set_is_urban(is_urban);
  }

  if (controller(kEdgeSpeedLimit)) {
    trip_edge->set_speed_limit(edgeinfo.speed_limit());
  }

  if (controller(kEdgeConditionalSpeedLimits)) {
    auto conditional_limits = edgeinfo.conditional_speed_limits();
    trip_edge->mutable_conditional_speed_limits()->Reserve(conditional_limits.size());
    for (const auto& limit : conditional_limits) {
      auto proto = trip_edge->mutable_conditional_speed_limits()->Add();
      proto->set_speed_limit(limit.speed_);

      auto* condition = proto->mutable_condition();

      switch (limit.td_.type()) {
        case kYMD:
          condition->set_day_dow_type(TripLeg_TimeDomain_DayDowType_kDayOfMonth);
          break;
        case kNthDow:
          condition->set_day_dow_type(TripLeg_TimeDomain_DayDowType_kNthDayOfWeek);
          break;
      }

      condition->set_dow_mask(limit.td_.dow());
      condition->set_begin_hrs(limit.td_.begin_hrs());
      condition->set_begin_mins(limit.td_.begin_mins());
      condition->set_begin_month(limit.td_.begin_month());
      condition->set_begin_day_dow(limit.td_.begin_day_dow());
      condition->set_begin_week(limit.td_.begin_week());
      condition->set_end_hrs(limit.td_.end_hrs());
      condition->set_end_mins(limit.td_.end_mins());
      condition->set_end_month(limit.td_.end_month());
      condition->set_end_day_dow(limit.td_.end_day_dow());
      condition->set_end_week(limit.td_.end_week());
    }
  }

  if (controller(kEdgeDefaultSpeed)) {
    trip_edge->set_default_speed(directededge->speed());
  }

  if (controller(kEdgeTruckSpeed)) {
    trip_edge->set_truck_speed(directededge->truck_speed());
  }

  if (directededge->truck_route() && controller(kEdgeTruckRoute)) {
    trip_edge->set_truck_route(true);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process transit information
  if (trip_id && (directededge->use() == Use::kRail || directededge->use() == Use::kBus)) {

    TransitRouteInfo* transit_route_info = trip_edge->mutable_transit_route_info();

    // Set block_id if requested
    if (controller(kEdgeTransitRouteInfoBlockId)) {
      transit_route_info->set_block_id(block_id);
    }

    // Set trip_id if requested
    if (controller(kEdgeTransitRouteInfoTripId)) {
      transit_route_info->set_trip_id(trip_id);
    }

    const TransitDeparture* transit_departure =
        graphtile->GetTransitDeparture(directededge->lineid(), trip_id,
                                       time_info.second_of_week % kSecondsPerDay);

    if (transit_departure) {

      // Set headsign if requested
      if (controller(kEdgeTransitRouteInfoHeadsign) && transit_departure->headsign_offset()) {
        transit_route_info->set_headsign(graphtile->GetName(transit_departure->headsign_offset()));
      }

      const TransitRoute* transit_route = graphtile->GetTransitRoute(transit_departure->routeindex());

      if (transit_route) {
        // Set transit type if requested
        if (controller(kEdgeTransitType)) {
          trip_edge->set_transit_type(GetTripLegTransitType(transit_route->route_type()));
        }

        // Set onestop_id if requested
        if (controller(kEdgeTransitRouteInfoOnestopId) && transit_route->one_stop_offset()) {
          transit_route_info->set_onestop_id(graphtile->GetName(transit_route->one_stop_offset()));
        }

        // Set short_name if requested
        if (controller(kEdgeTransitRouteInfoShortName) && transit_route->short_name_offset()) {
          transit_route_info->set_short_name(graphtile->GetName(transit_route->short_name_offset()));
        }

        // Set long_name if requested
        if (controller(kEdgeTransitRouteInfoLongName) && transit_route->long_name_offset()) {
          transit_route_info->set_long_name(graphtile->GetName(transit_route->long_name_offset()));
        }

        // Set color if requested
        if (controller(kEdgeTransitRouteInfoColor)) {
          transit_route_info->set_color(transit_route->route_color());
        }

        // Set text_color if requested
        if (controller(kEdgeTransitRouteInfoTextColor)) {
          transit_route_info->set_text_color(transit_route->route_text_color());
        }

        // Set description if requested
        if (controller(kEdgeTransitRouteInfoDescription) && transit_route->desc_offset()) {
          transit_route_info->set_description(graphtile->GetName(transit_route->desc_offset()));
        }

        // Set operator_onestop_id if requested
        if (controller(kEdgeTransitRouteInfoOperatorOnestopId) &&
            transit_route->op_by_onestop_id_offset()) {
          transit_route_info->set_operator_onestop_id(
              graphtile->GetName(transit_route->op_by_onestop_id_offset()));
        }

        // Set operator_name if requested
        if (controller(kEdgeTransitRouteInfoOperatorName) && transit_route->op_by_name_offset()) {
          transit_route_info->set_operator_name(
              graphtile->GetName(transit_route->op_by_name_offset()));
        }

        // Set operator_url if requested
        if (controller(kEdgeTransitRouteInfoOperatorUrl) && transit_route->op_by_website_offset()) {
          transit_route_info->set_operator_url(
              graphtile->GetName(transit_route->op_by_website_offset()));
        }
      }
    }
  }

  return trip_edge;
}

/**
 * This adds cost information at every node using supplementary costings provided at request time
 * There are some limitations here:
 * For multipoint routes the date_time used will not reflect the time offset that would have been if
 * you used the supplementary costing instead it is using the time at which the primary costing
 * arrived at the start of the leg
 * The same limitation is also true for arrive by routes in which the start time of the leg will be
 * the start time computed via the time offset from the primary costings time estimation
 * @param options     the api request options
 * @param src_pct     percent along the first edge of the path the start location snapped
 * @param tgt_pct     percent along the last edge of the path the end location snapped
 * @param time_info   the time tracking information representing the local time before
 *                    traversing the first edge
 * @param invariant   static date_time, dont offset the time as the path lengthens
 * @param reader      graph reader for tile access
 * @param leg         the already constructed trip leg to which extra cost information is added
 */
// TODO: care about the src and tgt pct per edge not just on first and last edges
void AccumulateRecostingInfoForward(const valhalla::Options& options,
                                    float src_pct,
                                    float tgt_pct,
                                    const baldr::TimeInfo& time_info,
                                    const bool invariant,
                                    valhalla::baldr::GraphReader& reader,
                                    valhalla::TripLeg& leg) {
  // bail if this is empty for some reason
  if (leg.node_size() == 0) {
    return;
  }

  // setup a callback for the recosting to get each edge
  auto in_itr = leg.node().begin();
  sif::EdgeCallback edge_cb = [&in_itr]() -> baldr::GraphId {
    auto edge_id = in_itr->has_edge() ? baldr::GraphId(in_itr->edge().id()) : baldr::GraphId{};
    ++in_itr;
    return edge_id;
  };

  // setup a callback for the recosting to tell us about the new label each made
  auto out_itr = leg.mutable_node()->begin();
  sif::LabelCallback label_cb = [&out_itr](const sif::PathEdgeLabel& label) -> void {
    // get the turn cost at this node
    out_itr->mutable_recosts()->rbegin()->mutable_transition_cost()->set_seconds(
        label.transition_cost().secs);
    out_itr->mutable_recosts()->rbegin()->mutable_transition_cost()->set_cost(
        label.transition_cost().cost);
    // get the elapsed time at the end of this labels edge and hang it on the next node
    ++out_itr;
    out_itr->mutable_recosts()->Add()->mutable_elapsed_cost()->set_seconds(label.cost().secs);
    out_itr->mutable_recosts()->rbegin()->mutable_elapsed_cost()->set_cost(label.cost().cost);
  };

  // do each recosting
  sif::CostFactory factory;
  for (const auto& recosting : options.recostings()) {
    // get the costing
    auto costing = factory.Create(recosting);
    // reset to the beginning of the route
    in_itr = leg.node().begin();
    out_itr = leg.mutable_node()->begin();
    // no elapsed time yet at the start of the leg
    out_itr->mutable_recosts()->Add()->mutable_elapsed_cost()->set_seconds(0);
    out_itr->mutable_recosts()->rbegin()->mutable_elapsed_cost()->set_cost(0);
    // do the recosting for this costing
    try {
      sif::recost_forward(reader, *costing, edge_cb, label_cb, src_pct, tgt_pct, time_info,
                          invariant);
      // no turn cost at the end of the leg
      out_itr->mutable_recosts()->rbegin()->mutable_transition_cost()->set_seconds(0);
      out_itr->mutable_recosts()->rbegin()->mutable_transition_cost()->set_cost(0);
    } // couldnt be recosted (difference in access for example) so we fill it with nulls to show this
    catch (...) {
      int should_have = leg.node(0).recosts_size();
      for (auto& node : *leg.mutable_node()) {
        if (node.recosts_size() == should_have) {
          node.mutable_recosts()->RemoveLast();
        }
        node.mutable_recosts()->Add();
      }
    }
  }
}

} // namespace

namespace valhalla {
namespace thor {

void TripLegBuilder::Build(
    const valhalla::Options& options,
    const AttributesController& controller,
    GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const std::vector<PathInfo>::const_iterator path_begin,
    const std::vector<PathInfo>::const_iterator path_end,
    valhalla::Location& origin,
    valhalla::Location& dest,
    TripLeg& trip_path,
    const std::vector<std::string>& algorithms,
    const std::function<void()>* interrupt_callback,
    const std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>>& edge_trimming,
    const std::vector<valhalla::Location>& intermediates) {
  // Test interrupt prior to building trip path
  if (interrupt_callback) {
    (*interrupt_callback)();
  }

  // Remember what algorithms were used to create this leg
  *trip_path.mutable_algorithms() = {algorithms.begin(), algorithms.end()};

  // Set origin, any through locations, and destination. Origin and
  // destination are assumed to be breaks.
  CopyLocations(trip_path, origin, intermediates, dest, path_begin, path_end);
  auto* tp_orig = trip_path.mutable_location(0);
  auto* tp_dest = trip_path.mutable_location(trip_path.location_size() - 1);

  // Keep track of the time
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  const auto forward_time_info = baldr::TimeInfo::make(origin, graphreader, &tz_cache);

  // check if we should use static time or offset time as the path lengthens
  const bool invariant = options.date_time_type() == Options::invariant;

  // Create an array of travel types per mode
  uint8_t travel_types[4];
  for (uint32_t i = 0; i < 4; i++) {
    travel_types[i] = (mode_costing[i] != nullptr) ? mode_costing[i]->travel_type() : 0;
  }

  // Get the first nodes graph id by using the end node of the first edge to get the tile with the
  // opposing edge then use the opposing index to get the opposing edge, and its end node is the
  // begin node of the original edge
  auto begin_tile = graphreader.GetGraphTile(path_begin->edgeid);
  if (begin_tile == nullptr) {
    throw tile_gone_error_t("TripLegBuilder::Build failed", path_begin->edgeid);
  }
  const auto* first_edge = begin_tile->directededge(path_begin->edgeid);
  auto first_tile = graphreader.GetGraphTile(first_edge->endnode());
  if (first_tile == nullptr) {
    throw tile_gone_error_t("TripLegBuilder::Build failed", first_edge->endnode());
  }
  auto* first_node = first_tile->node(first_edge->endnode());
  GraphId startnode =
      first_tile->directededge(first_node->edge_index() + first_edge->opp_index())->endnode();

  // Partial edge at the start and side of street (sos)
  float start_pct = 0.;
  valhalla::Location::SideOfStreet start_sos =
      valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone;
  PointLL start_vrt;
  for (const auto& e : origin.correlation().edges()) {
    if (e.graph_id() == path_begin->edgeid) {
      start_pct = e.percent_along();
      start_sos = e.side_of_street();
      start_vrt = PointLL(e.ll().lng(), e.ll().lat());
      break;
    }
  }

  // Set the origin projected location
  LatLng* proj_ll = tp_orig->mutable_correlation()->mutable_projected_ll();
  proj_ll->set_lat(start_vrt.lat());
  proj_ll->set_lng(start_vrt.lng());

  // Set the origin side of street, if one exists
  if (start_sos != valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone) {
    tp_orig->set_side_of_street(GetTripLegSideOfStreet(start_sos));
  }

  // Partial edge at the end
  float end_pct = 1.;
  valhalla::Location::SideOfStreet end_sos =
      valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone;
  PointLL end_vrt;
  for (const auto& e : dest.correlation().edges()) {
    if (e.graph_id() == (path_end - 1)->edgeid) {
      end_pct = e.percent_along();
      end_sos = e.side_of_street();
      end_vrt = PointLL(e.ll().lng(), e.ll().lat());
      break;
    }
  }

  // Set the destination projected location
  proj_ll = tp_dest->mutable_correlation()->mutable_projected_ll();
  proj_ll->set_lat(end_vrt.lat());
  proj_ll->set_lng(end_vrt.lng());

  // Set the destination side of street, if one exists
  if (end_sos != valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone) {
    tp_dest->set_side_of_street(GetTripLegSideOfStreet(end_sos));
  }

  // Structures to process admins
  std::unordered_map<AdminInfo, uint32_t, AdminInfo::AdminInfoHasher> admin_info_map;
  std::vector<AdminInfo> admin_info_list;

  // Iterate through path
  uint32_t prior_opp_local_index = -1;
  std::vector<PointLL> trip_shape;
  uint64_t osmchangeset = 0;
  size_t edge_index = 0;
  const DirectedEdge* prev_de = nullptr;
  graph_tile_ptr graphtile = nullptr;
  TimeInfo time_info = forward_time_info;
  // remember that MultimodalBuilder keeps 'time_info' as reference,
  // so we should care about 'time_info' updates during iterations
  MultimodalBuilder multimodal_builder(origin, time_info);

  // prepare to make some edges!
  trip_path.mutable_node()->Reserve((path_end - path_begin) + 1);

  // we track the intermediate locations while we iterate so we can update their shape index
  // from the edge index that we assigned to them earlier in route_action
  auto intermediate_itr = trip_path.mutable_location()->begin() + 1;
  double total_distance = 0;
  bool has_toll = false;
  bool has_ferry = false;
  bool has_highway = false;

  // loop over the edges to build the trip leg
  for (auto edge_itr = path_begin; edge_itr != path_end; ++edge_itr, ++edge_index) {
    const GraphId& edge = edge_itr->edgeid;
    graphtile = graphreader.GetGraphTile(edge, graphtile);
    if (graphtile == nullptr) {
      throw tile_gone_error_t("TripLegBuilder::Build failed", edge);
    }
    const DirectedEdge* directededge = graphtile->directededge(edge);
    const sif::TravelMode mode = edge_itr->mode;
    const uint8_t travel_type = travel_types[static_cast<uint32_t>(mode)];
    const auto& costing = mode_costing[static_cast<uint32_t>(mode)];

    if (directededge->toll()) {
      has_toll = true;
    }
    if (directededge->use() == Use::kFerry) {
      has_ferry = true;
    }
    if (directededge->classification() == baldr::RoadClass::kMotorway) {
      has_highway = true;
    }

    // Set node attributes - only set if they are true since they are optional
    graph_tile_ptr start_tile = graphtile;
    graphreader.GetGraphTile(startnode, start_tile);
    if (start_tile == nullptr) {
      throw tile_gone_error_t("TripLegBuilder::Build failed", startnode);
    }
    const NodeInfo* node = start_tile->node(startnode);

    if (osmchangeset == 0 && controller(kOsmChangeset)) {
      osmchangeset = start_tile->header()->dataset_id();
    }

    const bool is_first_edge = edge_itr == path_begin;
    const bool is_last_edge = edge_itr == (path_end - 1);

    // have to always compute the offset in case the timezone changes along the path
    // we could cache the timezone and just add seconds when the timezone doesnt change
    const float seconds_offset =
        (is_first_edge || invariant) ? 0.f : std::prev(edge_itr)->elapsed_cost.secs;
    time_info = forward_time_info.forward(seconds_offset, static_cast<int>(node->timezone()));

    // Add a node to the trip path and set its attributes.
    TripLeg_Node* trip_node = trip_path.add_node();

    if (controller(kNodeType)) {
      trip_node->set_type(GetTripLegNodeType(node->type()));
      if (node->traffic_signal())
        trip_node->set_traffic_signal(true);
    }

    if (node->intersection() == IntersectionType::kFork) {
      if (controller(kNodeFork)) {
        trip_node->set_fork(true);
      }
    }

    // Assign the elapsed time from the start of the leg
    if (controller(kNodeElapsedTime)) {
      if (edge_itr == path_begin) {
        trip_node->mutable_cost()->mutable_elapsed_cost()->set_seconds(0);
        trip_node->mutable_cost()->mutable_elapsed_cost()->set_cost(0);
      } else {
        trip_node->mutable_cost()->mutable_elapsed_cost()->set_seconds(
            std::prev(edge_itr)->elapsed_cost.secs);
        trip_node->mutable_cost()->mutable_elapsed_cost()->set_cost(
            std::prev(edge_itr)->elapsed_cost.cost);
      }
    }

    // Assign the admin index
    if (controller(kNodeAdminIndex)) {
      trip_node->set_admin_index(
          GetAdminIndex(start_tile->admininfo(node->admin_index()), admin_info_map, admin_info_list));
    }

    if (controller(kNodeTimeZone)) {
      auto tz = DateTime::get_tz_db().from_index(node->timezone());
      if (tz) {
        trip_node->set_time_zone(tz->name());
      }
    }

    if (controller(kNodeTransitionTime)) {
      trip_node->mutable_cost()->mutable_transition_cost()->set_seconds(
          edge_itr->transition_cost.secs);
      trip_node->mutable_cost()->mutable_transition_cost()->set_cost(edge_itr->transition_cost.cost);
    }

    // Add multi modal stuff
    multimodal_builder.Build(trip_node, edge_itr->trip_id, node, startnode, directededge, edge,
                             start_tile, graphtile, mode_costing, controller, graphreader);

    // Add edge to the trip node and set its attributes
    TripLeg_Edge* trip_edge =
        AddTripEdge(controller, edge, edge_itr->trip_id, multimodal_builder.block_id, mode,
                    travel_type, costing, directededge, node->drive_on_right(), trip_node, graphtile,
                    time_info, startnode.id(), node->named_intersection(), start_tile,
                    edge_itr->restriction_index, edge_itr->elapsed_cost.secs,
                    travel_type == PedestrianType::kBlind && mode == sif::TravelMode::kPedestrian);

    // some information regarding shape/length trimming
    float trim_start_pct = is_first_edge ? start_pct : 0;
    float trim_end_pct = is_last_edge ? end_pct : 1;

    // Some edges at the beginning and end of the path and at intermediate locations will need trimmed
    uint32_t begin_index = is_first_edge ? 0 : trip_shape.size() - 1;
    auto edgeinfo = graphtile->edgeinfo(directededge);
    auto trimming = edge_trimming.end();
    if (!edge_trimming.empty() &&
        (trimming = edge_trimming.find(edge_index)) != edge_trimming.end()) {
      // Get edge shape and reverse it if directed edge is not forward.
      auto edge_shape = edgeinfo.shape();
      if (!directededge->forward()) {
        std::reverse(edge_shape.begin(), edge_shape.end());
      }

      // Grab the edge begin and end info
      const auto& edge_begin_info = trimming->second.first;
      const auto& edge_end_info = trimming->second.second;

      // Start by assuming no trimming
      double begin_trim_dist = 0, end_trim_dist = 1;
      auto begin_trim_vrt = edge_shape.front(), end_trim_vrt = edge_shape.back();

      // Trimming needed
      if (edge_begin_info.trim) {
        begin_trim_dist = edge_begin_info.distance_along;
        begin_trim_vrt = edge_begin_info.vertex;
      }
      // Handle partial shape for first edge
      else if (is_first_edge && !edge_begin_info.trim) {
        begin_trim_dist = start_pct;
        begin_trim_vrt = start_vrt;
      }

      // Trimming needed
      if (edge_end_info.trim) {
        end_trim_dist = edge_end_info.distance_along;
        end_trim_vrt = edge_end_info.vertex;
      } // Handle partial shape for last edge
      else if (is_last_edge && !edge_end_info.trim) {
        end_trim_dist = end_pct;
        end_trim_vrt = end_vrt;
      }

      // Overwrite the trimming information for the edge length now that we know what it is
      trim_start_pct = begin_trim_dist;
      trim_end_pct = end_trim_dist;

      // Trim the shape
      auto edge_length = static_cast<float>(directededge->length());
      trim_shape(begin_trim_dist * edge_length, begin_trim_vrt, end_trim_dist * edge_length,
                 end_trim_vrt, edge_shape);
      // Add edge shape to the trip and skip the first point when its redundant with the previous edge
      trip_shape.insert(trip_shape.end(), edge_shape.begin() + !is_first_edge, edge_shape.end());
    } // We need to clip the shape if its at the beginning or end
    else if (is_first_edge || is_last_edge) {
      // Get edge shape and reverse it if directed edge is not forward.
      auto edge_shape = edgeinfo.shape();
      if (!directededge->forward()) {
        std::reverse(edge_shape.begin(), edge_shape.end());
      }
      float total = static_cast<float>(directededge->length());
      // Trim both ways
      if (is_first_edge && is_last_edge) {
        trim_shape(start_pct * total, start_vrt, end_pct * total, end_vrt, edge_shape);
      } // Trim the shape at the front for the first edge
      else if (is_first_edge) {
        trim_shape(start_pct * total, start_vrt, total, edge_shape.back(), edge_shape);
      } // And at the back if its the last edge
      else {
        trim_shape(0, edge_shape.front(), end_pct * total, end_vrt, edge_shape);
      }
      // Keep the shape
      trip_shape.insert(trip_shape.end(), edge_shape.begin() + !is_first_edge, edge_shape.end());
    } // Just get the shape in there in the right direction no clipping needed
    else {
      if (directededge->forward()) {
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().begin() + 1, edgeinfo.shape().end());
      } else {
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().rbegin() + 1, edgeinfo.shape().rend());
      }
    }

    // Set the portion of the edge we used
    // TODO: attributes controller and then use this in recosting
    trip_edge->set_source_along_edge(trim_start_pct);
    trip_edge->set_target_along_edge(trim_end_pct);

    // We need the total offset from the beginning of leg for the intermediate locations
    auto previous_total_distance = total_distance;
    total_distance += directededge->length() * (trim_end_pct - trim_start_pct);

    // If we are at a node or if we hit the edge index that matches our through location edge index,
    // we need to reset to the shape index then increment the iterator
    if (intermediate_itr != trip_path.mutable_location()->end() &&
        intermediate_itr->correlation().leg_shape_index() == edge_index) {
      intermediate_itr->mutable_correlation()->set_leg_shape_index(trip_shape.size() - 1);
      intermediate_itr->mutable_correlation()->set_distance_from_leg_origin(total_distance);
      // NOTE:
      // So for intermediate locations that dont have any trimming we know they occur at the node
      // In this case and only for ARRIVE_BY, the edge index that we convert to shape is off by 1
      // So here we need to set this one as if it were at the end of the previous edge in the path
      if (trimming == edge_trimming.end() && options.date_time_type() == Options::arrive_by) {
        intermediate_itr->mutable_correlation()->set_leg_shape_index(begin_index);
        intermediate_itr->mutable_correlation()->set_distance_from_leg_origin(
            previous_total_distance);
      }
      ++intermediate_itr;
    }

    // Set length if requested. Convert to km
    if (controller(kEdgeLength)) {
      float km =
          std::max(directededge->length() * kKmPerMeter * (trim_end_pct - trim_start_pct), 0.0f);
      trip_edge->set_length_km(km);
    }

    // How long on this edge?
    auto edge_seconds = edge_itr->elapsed_cost.secs - edge_itr->transition_cost.secs;
    if (edge_itr != path_begin)
      edge_seconds -= std::prev(edge_itr)->elapsed_cost.secs;

    // Set shape attributes, sending incidents enables them in the pbf
    auto incidents = controller(kIncidents) ? graphreader.GetIncidents(edge_itr->edgeid, graphtile)
                                            : valhalla::baldr::IncidentResult{};

    graph_tile_ptr end_node_tile = graphtile;
    graphreader.GetGraphTile(directededge->endnode(), end_node_tile);
    SetShapeAttributes(controller, graphtile, end_node_tile, directededge, trip_shape, begin_index,
                       trip_path, trim_start_pct, trim_end_pct, edge_seconds,
                       costing->flow_mask() & kCurrentFlowMask, incidents);

    // Set begin shape index if requested
    if (controller(kEdgeBeginShapeIndex)) {
      trip_edge->set_begin_shape_index(begin_index);
    }

    // Set end shape index if requested
    if (controller(kEdgeEndShapeIndex)) {
      trip_edge->set_end_shape_index(trip_shape.size() - 1);
    }

    // Set begin and end heading if requested. Uses trip_shape so
    // must be done after the edge's shape has been added.
    SetHeadings(trip_edge, controller, directededge, trip_shape, begin_index);

    // Add elevation along the edge if requested
    if (controller(kEdgeElevation)) {
      SetElevation(trip_edge, trim_start_pct, trim_end_pct, node, directededge, graphtile,
                   graphreader);
    }

    // Add landmarks in the directededge to the trip leg
    AddLandmarks(edgeinfo, trip_edge, controller, directededge, trip_shape, begin_index);

    // Add the intersecting edges at the node. Skip it if the node was an inner node (excluding start
    // node and end node) of a shortcut that was recovered.
    if (startnode.Is_Valid() && !edge_itr->start_node_is_recovered) {
      AddIntersectingEdges(controller, start_tile, node, directededge, prev_de, prior_opp_local_index,
                           graphreader, trip_node,
                           travel_type == PedestrianType::kBlind &&
                               mode == sif::TravelMode::kPedestrian);
    }

    ////////////// Prepare for the next iteration

    // Set the endnode of this directed edge as the startnode of the next edge.
    startnode = directededge->endnode();

    // Save the opposing edge as the previous DirectedEdge (for name consistency)
    if (!directededge->IsTransitLine()) {
      graph_tile_ptr t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : graphtile;
      if (t2 == nullptr) {
        continue;
      }
      GraphId oppedge = t2->GetOpposingEdgeId(directededge);
      prev_de = t2->directededge(oppedge);
    }

    // Save the index of the opposing local directed edge at the end node
    prior_opp_local_index = directededge->opp_local_idx();
  }

  // Add the last node
  auto* node = trip_path.add_node();
  if (controller(kNodeAdminIndex)) {
    auto last_tile = graphreader.GetGraphTile(startnode);
    if (last_tile == nullptr) {
      throw tile_gone_error_t("TripLegBuilder::Build failed", startnode);
    }
    node->set_admin_index(
        GetAdminIndex(last_tile->admininfo(last_tile->node(startnode)->admin_index()), admin_info_map,
                      admin_info_list));
  }
  if (controller(kNodeElapsedTime)) {
    node->mutable_cost()->mutable_elapsed_cost()->set_seconds(std::prev(path_end)->elapsed_cost.secs);
    node->mutable_cost()->mutable_elapsed_cost()->set_cost(std::prev(path_end)->elapsed_cost.cost);
  }

  if (controller(kNodeTransitionTime)) {
    node->mutable_cost()->mutable_transition_cost()->set_seconds(0);
    node->mutable_cost()->mutable_transition_cost()->set_cost(0);
  }

  if (controller(kShapeAttributesClosure)) {
    // Set the end shape index if we're ending on a closure as the last index is
    // not processed in SetShapeAttributes above
    valhalla::TripLeg_Closure* closure = fetch_last_closure_annotation(trip_path);
    if (closure && !closure->has_end_shape_index_case()) {
      closure->set_end_shape_index(trip_shape.size() - 1);
    }
  }

  // Assign the admins
  AssignAdmins(controller, trip_path, admin_info_list);

  // Set the bounding box of the shape
  SetBoundingBox(trip_path, trip_shape);

  // Set shape if requested
  if (controller(kShape)) {
    trip_path.set_shape(encode<std::vector<PointLL>>(trip_shape));
  }

  if (osmchangeset != 0 && controller(kOsmChangeset)) {
    trip_path.set_osm_changeset(osmchangeset);
  }

  Summary* summary = trip_path.mutable_summary();
  summary->set_has_toll(has_toll);
  summary->set_has_ferry(has_ferry);
  summary->set_has_highway(has_highway);

  // Add that extra costing information if requested
  AccumulateRecostingInfoForward(options, start_pct, end_pct, forward_time_info, invariant,
                                 graphreader, trip_path);
}

} // namespace thor
} // namespace valhalla
