using System;
using System.Collections.Generic;

namespace valhalla.baldr
{
    public class AttributesController
    {
        public static readonly Dictionary<string, bool> kDefaultAttributes = new Dictionary<string, bool>
        {
            // Edge keys
            { "edge_names", true },
            { "edge_length", true },
            { "edge_speed", true },
            { "edge_road_class", true },
            { "edge_begin_heading", true },
            { "edge_end_heading", true },
            { "edge_begin_shape_index", true },
            { "edge_end_shape_index", true },
            { "edge_traversability", true },
            { "edge_use", true },
            { "edge_toll", true },
            { "edge_unpaved", true },
            { "edge_tunnel", true },
            { "edge_bridge", true },
            { "edge_roundabout", true },
            { "edge_internal_intersection", true },
            { "edge_drive_on_right", true },
            { "edge_surface", true },
            { "edge_sign_exit_number", true },
            { "edge_sign_exit_branch", true },
            { "edge_sign_exit_toward", true },
            { "edge_sign_exit_name", true },
            { "edge_sign_guide_branch", true },
            { "edge_sign_guide_toward", true },
            { "edge_sign_junction_name", true },
            { "edge_sign_guidance_view_junction", true },
            { "edge_sign_guidance_view_signboard", true },
            { "edge_travel_mode", true },
            { "edge_vehicle_type", true },
            { "edge_pedestrian_type", true },
            { "edge_bicycle_type", true },
            { "edge_transit_type", true },
            { "edge_transit_route_info_onestop_id", true },
            { "edge_transit_route_info_block_id", true },
            { "edge_transit_route_info_trip_id", true },
            { "edge_transit_route_info_short_name", true },
            { "edge_transit_route_info_long_name", true },
            { "edge_transit_route_info_headsign", true },
            { "edge_transit_route_info_color", true },
            { "edge_transit_route_info_text_color", true },
            { "edge_transit_route_info_description", true },
            { "edge_transit_route_info_operator_onestop_id", true },
            { "edge_transit_route_info_operator_name", true },
            { "edge_transit_route_info_operator_url", true },
            { "edge_id", true },
            { "edge_way_id", true },
            { "edge_weighted_grade", true },
            { "edge_max_upward_grade", true },
            { "edge_max_downward_grade", true },
            { "edge_mean_elevation", true },
            { "edge_lane_count", true },
            { "edge_lane_connectivity", true },
            { "edge_cycle_lane", true },
            { "edge_bicycle_network", true },
            { "edge_elevation", false },
            { "edge_sac_scale", true },
            { "edge_shoulder", true },
            { "edge_sidewalk", true },
            { "edge_density", true },
            { "edge_speed_limit", true },
            { "edge_conditional_speed_limits", true },
            { "edge_truck_speed", true },
            { "edge_truck_route", true },
            { "edge_default_speed", true },
            { "edge_destination_only", true },
            { "edge_is_urban", false },
            { "edge_tagged_values", true },
            { "edge_indoor", true },
            { "edge_landmarks", true },
            { "edge_country_crossing", true },
            { "edge_forward", true },
            { "edge_levels", true },

            // Node keys
            { "incidents", false },
            { "node_intersecting_edge_begin_heading", true },
            { "node_intersecting_edge_from_edge_name_consistency", true },
            { "node_intersecting_edge_to_edge_name_consistency", true },
            { "node_intersecting_edge_driveability", true },
            { "node_intersecting_edge_cyclability", true },
            { "node_intersecting_edge_walkability", true },
            { "node_intersecting_edge_use", true },
            { "node_intersecting_edge_road_class", true },
            { "node_intersecting_edge_lane_count", true },
            { "node_intersecting_edge_sign_info", true },
            { "node_elapsed_time", true },
            { "node_admin_index", true },
            { "node_type", true },
            { "node_fork", true },
            { "node_transit_platform_info_type", true },
            { "node_transit_platform_info_onestop_id", true },
            { "node_transit_platform_info_name", true },
            { "node_transit_platform_info_station_onestop_id", true },
            { "node_transit_platform_info_station_name", true },
            { "node_transit_platform_info_arrival_date_time", true },
            { "node_transit_platform_info_departure_date_time", true },
            { "node_transit_platform_info_is_parent_stop", true },
            { "node_transit_platform_info_assumed_schedule", true },
            { "node_transit_platform_info_lat_lon", true },
            { "node_transit_station_info_onestop_id", true },
            { "node_transit_station_info_name", true },
            { "node_transit_station_info_lat_lon", true },
            { "node_transit_egress_info_onestop_id", true },
            { "node_transit_egress_info_name", true },
            { "node_transit_egress_info_lat_lon", true },
            { "node_time_zone", true },
            { "node_transition_time", true },

            // Top level: admin list, full shape, and shape bounding box keys
            { "osm_changeset", true },
            { "admin_country_code", true },
            { "admin_country_text", true },
            { "admin_state_code", true },
            { "admin_state_text", true },
            { "shape", true },
            { "matched_point", true },
            { "matched_type", true },
            { "matched_edge_index", true },
            { "matched_begin_route_discontinuity", true },
            { "matched_end_route_discontinuity", true },
            { "matched_distance_along_edge", true },
            { "matched_distance_from_trace_point", true },
            { "confidence_score", true },
            { "raw_score", true },

            // Per-shape attributes
            { "shape_attributes_time", false },
            { "shape_attributes_length", false },
            { "shape_attributes_speed", false },
            { "shape_attributes_speed_limit", false },
            { "shape_attributes_closure", false },
        };

        private Dictionary<string, bool> attributes;

        public AttributesController()
        {
            attributes = new Dictionary<string, bool>(kDefaultAttributes);
        }

        public AttributesController(Options options, bool is_strict_filter)
        {
            attributes = new Dictionary<string, bool>(kDefaultAttributes);

            switch (options.filter_action())
            {
                case FilterAction.include:
                    if (is_strict_filter)
                        disable_all();
                    foreach (var filter_attribute in options.filter_attributes())
                    {
                        if (attributes.ContainsKey(filter_attribute))
                        {
                            attributes[filter_attribute] = true;
                        }
                        else
                        {
                            Console.Error.WriteLine("Invalid filter attribute " + filter_attribute);
                        }
                    }
                    break;
                case FilterAction.exclude:
                    foreach (var filter_attribute in options.filter_attributes())
                    {
                        if (attributes.ContainsKey(filter_attribute))
                        {
                            attributes[filter_attribute] = false;
                        }
                        else
                        {
                            Console.Error.WriteLine("Invalid filter attribute " + filter_attribute);
                        }
                    }
                    break;
                default:
                    break;
            }

            attributes["edge_elevation"] = options.elevation_interval() > 0.0f;
        }

        public void disable_all()
        {
            var keys = new List<string>(attributes.Keys);
            foreach (var key in keys)
            {
                attributes[key] = false;
            }
        }

        public bool this[string key]
        {
            get { return attributes[key]; }
        }

        public bool category_attribute_enabled(string category)
        {
            foreach (var pair in attributes)
            {
                if (pair.Key.StartsWith(category) && pair.Value)
                {
                    return true;
                }
            }
            return false;
        }
    }
}
