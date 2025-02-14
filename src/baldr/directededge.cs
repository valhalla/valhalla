using System;
using System.Collections.Generic;
using System.Text.Json;

namespace valhalla.baldr
{
    public class DirectedEdge
    {
        public uint endnode { get; set; }
        public uint speed_ { get; set; }
        public uint speed_type_ { get; set; }
        public uint free_flow_speed_ { get; set; }
        public uint constrained_flow_speed_ { get; set; }
        public bool has_predicted_speed_ { get; set; }
        public uint opp_index_ { get; set; }
        public uint edgeinfo_offset_ { get; set; }
        public uint access_restriction_ { get; set; }
        public uint start_restriction_ { get; set; }
        public uint end_restriction_ { get; set; }
        public bool complex_restriction_ { get; set; }
        public bool sign_ { get; set; }
        public bool toll_ { get; set; }
        public bool seasonal_ { get; set; }
        public bool dest_only_ { get; set; }
        public bool tunnel_ { get; set; }
        public bool bridge_ { get; set; }
        public bool roundabout_ { get; set; }
        public bool traffic_signal_ { get; set; }
        public bool forward_ { get; set; }
        public bool not_thru_ { get; set; }
        public bool stop_sign_ { get; set; }
        public bool yield_sign_ { get; set; }
        public uint cycle_lane_ { get; set; }
        public bool bike_network_ { get; set; }
        public bool truck_route_ { get; set; }
        public uint lanecount_ { get; set; }
        public bool ctry_crossing_ { get; set; }
        public bool sidewalk_left_ { get; set; }
        public bool sidewalk_right_ { get; set; }
        public uint sac_scale_ { get; set; }
        public bool deadend_ { get; set; }
        public uint length_ { get; set; }
        public uint weighted_grade_ { get; set; }
        public uint max_up_slope_ { get; set; }
        public uint max_down_slope_ { get; set; }
        public uint curvature_ { get; set; }
        public uint forwardaccess_ { get; set; }
        public uint reverseaccess_ { get; set; }
        public uint classification_ { get; set; }
        public uint use_ { get; set; }
        public uint surface_ { get; set; }
        public bool link_ { get; set; }
        public bool internal_ { get; set; }
        public uint localedgeidx_ { get; set; }
        public uint opp_local_idx_ { get; set; }
        public uint shortcut_ { get; set; }
        public uint superseded_ { get; set; }
        public bool is_shortcut_ { get; set; }
        public bool leaves_tile_ { get; set; }
        public bool bss_connection_ { get; set; }
        public bool lit_ { get; set; }

        public DirectedEdge()
        {
            weighted_grade_ = 6;
        }

        public void set_endnode(uint endnode)
        {
            this.endnode = endnode;
        }

        public void set_free_flow_speed(uint speed)
        {
            if (speed > 255)
            {
                Console.WriteLine("Exceeding maximum.  Free flow speed: " + speed);
                free_flow_speed_ = 255;
            }
            else
            {
                free_flow_speed_ = speed;
            }
        }

        public void set_constrained_flow_speed(uint speed)
        {
            if (speed > 255)
            {
                Console.WriteLine("Exceeding maximum.  Constrained flow speed: " + speed);
                constrained_flow_speed_ = 255;
            }
            else
            {
                constrained_flow_speed_ = speed;
            }
        }

        public void set_has_predicted_speed(bool p)
        {
            has_predicted_speed_ = p;
        }

        public void set_edgeinfo_offset(uint offset)
        {
            if (offset > 16777215)
            {
                throw new Exception("DirectedEdge: exceeded maximum edgeinfo offset");
            }
            else
            {
                edgeinfo_offset_ = offset;
            }
        }

        public void set_access_restriction(uint access)
        {
            access_restriction_ = access;
        }

        public void set_sign(bool exit)
        {
            sign_ = exit;
        }

        public void set_length(uint length, bool should_error)
        {
            if (length > 16777215)
            {
                if (should_error)
                {
                    throw new Exception("DirectedEdgeBuilder: exceeded maximum edge length");
                }
                length_ = 16777215;
            }
            else
            {
                length_ = length;
            }
        }

        public void set_weighted_grade(uint factor)
        {
            if (factor > 15)
            {
                Console.WriteLine("Exceeding max. weighted grade factor: " + factor);
                weighted_grade_ = 6;
            }
            else
            {
                weighted_grade_ = factor;
            }
        }

        public void set_curvature(uint factor)
        {
            if (factor > 15)
            {
                Console.WriteLine("Exceeding max. curvature factor: " + factor);
                curvature_ = 0;
            }
            else
            {
                curvature_ = factor;
            }
        }

        public void set_laneconnectivity(bool lc)
        {
            // Not implemented in C# version
        }

        public void set_shoulder(bool shoulder)
        {
            // Not implemented in C# version
        }

        public void set_dismount(bool dismount)
        {
            // Not implemented in C# version
        }

        public void set_use_sidepath(bool use_sidepath)
        {
            // Not implemented in C# version
        }

        public void set_deadend(bool d)
        {
            deadend_ = d;
        }

        public void set_toll(bool toll)
        {
            toll_ = toll;
        }

        public void set_seasonal(bool seasonal)
        {
            seasonal_ = seasonal;
        }

        public void set_dest_only(bool destonly)
        {
            dest_only_ = destonly;
        }

        public void set_dest_only_hgv(bool destonly_hgv)
        {
            // Not implemented in C# version
        }

        public void set_tunnel(bool tunnel)
        {
            tunnel_ = tunnel;
        }

        public void set_bridge(bool bridge)
        {
            bridge_ = bridge;
        }

        public void set_indoor(bool indoor)
        {
            // Not implemented in C# version
        }

        public void set_hov_type(uint hov_type)
        {
            // Not implemented in C# version
        }

        public bool is_hov_only()
        {
            return (forwardaccess_ & 16) != 0 && (forwardaccess_ & 1) == 0;
        }

        public void set_roundabout(bool roundabout)
        {
            roundabout_ = roundabout;
        }

        public void set_traffic_signal(bool signal)
        {
            traffic_signal_ = signal;
        }

        public void set_stop_sign(bool sign)
        {
            stop_sign_ = sign;
        }

        public void set_yield_sign(bool sign)
        {
            yield_sign_ = sign;
        }

        public void set_forward(bool forward)
        {
            forward_ = forward;
        }

        public void set_not_thru(bool not_thru)
        {
            not_thru_ = not_thru;
        }

        public void set_opp_index(uint opp_index)
        {
            opp_index_ = opp_index;
        }

        public void set_cyclelane(uint cyclelane)
        {
            cycle_lane_ = cyclelane;
        }

        public void set_bike_network(bool bike_network)
        {
            bike_network_ = bike_network;
        }

        public void set_truck_route(bool truck_route)
        {
            truck_route_ = truck_route;
        }

        public void set_lanecount(uint lanecount)
        {
            if (lanecount > 15)
            {
                Console.WriteLine("Exceeding maximum lane count: " + lanecount);
                lanecount_ = 15;
            }
            else if (lanecount == 0)
            {
                lanecount_ = 1;
            }
            else
            {
                lanecount_ = lanecount;
            }
        }

        public void set_restrictions(uint mask)
        {
            if (mask >= (1 << 8))
            {
                Console.WriteLine("Restrictions mask exceeds allowable limit: " + mask);
                // Not implemented in C# version
            }
            else
            {
                // Not implemented in C# version
            }
        }

        public void set_use(uint use)
        {
            use_ = use;
        }

        public void set_speed_type(uint speed_type)
        {
            speed_type_ = speed_type;
        }

        public void set_ctry_crossing(bool crossing)
        {
            ctry_crossing_ = crossing;
        }

        public void set_forwardaccess(uint modes)
        {
            if (modes > 1023)
            {
                Console.WriteLine("DirectedEdge: forward access exceeds maximum allowed: " + modes);
                forwardaccess_ = (modes & 1023);
            }
            else
            {
                forwardaccess_ = modes;
            }
        }

        public void set_all_forward_access()
        {
            forwardaccess_ = 1023;
            reverseaccess_ = 1023;
        }

        public void set_reverseaccess(uint modes)
        {
            if (modes > 1023)
            {
                Console.WriteLine("DirectedEdge: reverse access exceeds maximum allowed: " + modes);
                reverseaccess_ = (modes & 1023);
            }
            else
            {
                reverseaccess_ = modes;
            }
        }

        public void set_speed(uint speed)
        {
            if (speed > 255)
            {
                Console.WriteLine("Exceeding maximum.  Average speed: " + speed);
                speed_ = 255;
            }
            else
            {
                speed_ = speed;
            }
        }

        public void set_truck_speed(uint speed)
        {
            if (speed > 255)
            {
                Console.WriteLine("Exceeding maximum.  Truck speed: " + speed);
                truck_speed_ = 255;
            }
            else
            {
                truck_speed_ = speed;
            }
        }

        public void set_classification(uint roadclass)
        {
            classification_ = roadclass;
        }

        public void set_sac_scale(uint sac_scale)
        {
            sac_scale_ = sac_scale;
        }

        public void set_name_consistency(uint idx, bool c)
        {
            // Not implemented in C# version
        }

        public void set_surface(uint surface)
        {
            surface_ = surface;
        }

        public void set_link(bool link)
        {
            link_ = link;
        }

        public void set_internal(bool internal)
        {
            internal_ = internal;
        }

        public void set_start_restriction(uint modes)
        {
            start_restriction_ = modes;
        }

        public void set_end_restriction(uint modes)
        {
            end_restriction_ = modes;
        }

        public void complex_restriction(bool part_of)
        {
            complex_restriction_ = part_of;
        }

        public void set_density(uint density)
        {
            if (density > 15)
            {
                Console.WriteLine("Exceeding max. density: " + density);
                // Not implemented in C# version
            }
            else
            {
                // Not implemented in C# version
            }
        }

        public void set_named(bool named)
        {
            // Not implemented in C# version
        }

        public void set_sidewalk_left(bool sidewalk)
        {
            sidewalk_left_ = sidewalk;
        }

        public void set_sidewalk_right(bool sidewalk)
        {
            sidewalk_right_ = sidewalk;
        }

        public void set_turntype(uint localidx, uint turntype)
        {
            // Not implemented in C# version
        }

        public void set_edge_to_left(uint localidx, bool left)
        {
            // Not implemented in C# version
        }

        public void set_stopimpact(uint localidx, uint stopimpact)
        {
            // Not implemented in C# version
        }

        public void set_lineid(uint lineid)
        {
            // Not implemented in C# version
        }

        public void set_edge_to_right(uint localidx, bool right)
        {
            // Not implemented in C# version
        }

        public void set_localedgeidx(uint idx)
        {
            if (idx > 255)
            {
                Console.WriteLine("Local Edge Index exceeds max: " + idx);
                localedgeidx_ = 255;
            }
            else
            {
                localedgeidx_ = idx;
            }
        }

        public void set_opp_local_idx(uint idx)
        {
            if (idx > 255)
            {
                Console.WriteLine("Exceeding max edges in opposing local index: " + idx);
                opp_local_idx_ = 255;
            }
            else
            {
                opp_local_idx_ = idx;
            }
        }

        public void set_shortcut(uint shortcut)
        {
            if (shortcut == 0)
            {
                Console.WriteLine("Invalid shortcut mask = 0");
                return;
            }

            if (shortcut <= 15)
            {
                shortcut_ = (1 << (int)(shortcut - 1));
            }

            is_shortcut_ = true;
        }

        public void set_superseded(uint superseded)
        {
            if (superseded > 15)
            {
                Console.WriteLine("Exceeding max shortcut edges from a node: " + superseded);
            }
            else if (superseded == 0)
            {
                superseded_ = 0;
            }
            else
            {
                superseded_ = (1 << (int)(superseded - 1));
            }
        }

        public void set_leaves_tile(bool leaves_tile)
        {
            leaves_tile_ = leaves_tile;
        }

        public void set_max_up_slope(float slope)
        {
            if (slope < 0.0f)
            {
                max_up_slope_ = 0;
            }
            else if (slope < 16.0f)
            {
                max_up_slope_ = (uint)Math.Ceiling(slope);
            }
            else if (slope < 76.0f)
            {
                max_up_slope_ = 0x10 | (uint)Math.Ceiling((slope - 16.0f) * 0.25f);
            }
            else
            {
                max_up_slope_ = 0x1f;
            }
        }

        public void set_max_down_slope(float slope)
        {
            if (slope > 0.0f)
            {
                max_down_slope_ = 0;
            }
            else if (slope > -16.0f)
            {
                max_down_slope_ = (uint)Math.Ceiling(-slope);
            }
            else if (slope > -76.0f)
            {
                max_down_slope_ = 0x10 | (uint)Math.Ceiling((-slope - 16.0f) * 0.25f);
            }
            else
            {
                max_down_slope_ = 0x1f;
            }
        }

        public void set_bss_connection(bool bss_connection)
        {
            bss_connection_ = bss_connection;
        }

        public void set_lit(bool lit)
        {
            lit_ = lit;
        }

        public JsonElement json()
        {
            var map = new Dictionary<string, object>
            {
                { "end_node", endnode },
                { "speeds", new Dictionary<string, object>
                    {
                        { "default", speed_ },
                        { "type", speed_type_ },
                        { "free_flow", free_flow_speed_ },
                        { "constrained_flow", constrained_flow_speed_ },
                        { "predicted", has_predicted_speed_ }
                    }
                },
                { "access_restriction", access_restriction_ != 0 },
                { "start_restriction", access_json(start_restriction_) },
                { "end_restriction", access_json(end_restriction_) },
                { "part_of_complex_restriction", complex_restriction_ },
                { "has_sign", sign_ },
                { "toll", toll_ },
                { "seasonal", seasonal_ },
                { "destination_only", dest_only_ },
                { "tunnel", tunnel_ },
                { "bridge", bridge_ },
                { "round_about", roundabout_ },
                { "traffic_signal", traffic_signal_ },
                { "forward", forward_ },
                { "not_thru", not_thru_ },
                { "stop_sign", stop_sign_ },
                { "yield_sign", yield_sign_ },
                { "cycle_lane", cycle_lane_ },
                { "bike_network", bike_network_ },
                { "truck_route", truck_route_ },
                { "lane_count", lanecount_ },
                { "country_crossing", ctry_crossing_ },
                { "sidewalk_left", sidewalk_left_ },
                { "sidewalk_right", sidewalk_right_ },
                { "sac_scale", sac_scale_ },
                { "deadend", deadend_ },
                { "geo_attributes", new Dictionary<string, object>
                    {
                        { "length", length_ },
                        { "weighted_grade", (double)(weighted_grade_ - 6.0) / 0.6 },
                        { "max_up_slope", (double)max_up_slope_ },
                        { "max_down_slope", (double)max_down_slope_ },
                        { "curvature", curvature_ }
                    }
                },
                { "access", access_json(forwardaccess_) },
                { "classification", new Dictionary<string, object>
                    {
                        { "classification", classification_ },
                        { "use", use_ },
                        { "surface", surface_ },
                        { "link", link_ },
                        { "internal", internal_ }
                    }
                }
            };

            if (is_hov_only())
            {
                map.Add("hov_type", "HOV");
            }

            return JsonDocument.Parse(JsonSerializer.Serialize(map)).RootElement;
        }

        private Dictionary<string, bool> access_json(uint access)
        {
            return new Dictionary<string, bool>
            {
                { "bicycle", (access & 1) != 0 },
                { "bus", (access & 2) != 0 },
                { "car", (access & 4) != 0 },
                { "emergency", (access & 8) != 0 },
                { "HOV", (access & 16) != 0 },
                { "pedestrian", (access & 32) != 0 },
                { "taxi", (access & 64) != 0 },
                { "truck", (access & 128) != 0 },
                { "wheelchair", (access & 256) != 0 },
                { "moped", (access & 512) != 0 },
                { "motorcycle", (access & 1024) != 0 }
            };
        }
    }
}
