using System;
using System.Collections.Generic;
using System.Text.Json;

namespace valhalla.baldr
{
    public class NodeInfo
    {
        public uint lat_offset_ { get; set; }
        public uint lon_offset_ { get; set; }
        public uint edge_index_ { get; set; }
        public uint edge_count_ { get; set; }
        public uint access_ { get; set; }
        public uint intersection_ { get; set; }
        public uint admin_index_ { get; set; }
        public uint timezone_ { get; set; }
        public uint timezone_ext_1_ { get; set; }
        public uint local_driveability_ { get; set; }
        public uint density_ { get; set; }
        public uint type_ { get; set; }
        public uint local_edge_count_ { get; set; }
        public bool drive_on_right_ { get; set; }
        public uint elevation_ { get; set; }
        public bool tagged_access_ { get; set; }
        public bool mode_change_ { get; set; }
        public bool named_ { get; set; }
        public bool traffic_signal_ { get; set; }
        public uint transition_index_ { get; set; }
        public uint headings_ { get; set; }

        public NodeInfo()
        {
            lat_offset_ = 0;
            lon_offset_ = 0;
            edge_index_ = 0;
            edge_count_ = 0;
            access_ = 0;
            intersection_ = 0;
            admin_index_ = 0;
            timezone_ = 0;
            timezone_ext_1_ = 0;
            local_driveability_ = 0;
            density_ = 0;
            type_ = 0;
            local_edge_count_ = 0;
            drive_on_right_ = false;
            elevation_ = 0;
            tagged_access_ = false;
            mode_change_ = false;
            named_ = false;
            traffic_signal_ = false;
            transition_index_ = 0;
            headings_ = 0;
        }

        public NodeInfo(PointLL tile_corner, PointLL ll, uint access, NodeType type, bool traffic_signal, bool tagged_access, bool private_access, bool cash_only_toll)
        {
            lat_offset_ = 0;
            lon_offset_ = 0;
            edge_index_ = 0;
            edge_count_ = 0;
            access_ = 0;
            intersection_ = 0;
            admin_index_ = 0;
            timezone_ = 0;
            timezone_ext_1_ = 0;
            local_driveability_ = 0;
            density_ = 0;
            type_ = 0;
            local_edge_count_ = 0;
            drive_on_right_ = false;
            elevation_ = 0;
            tagged_access_ = false;
            mode_change_ = false;
            named_ = false;
            traffic_signal_ = false;
            transition_index_ = 0;
            headings_ = 0;

            set_latlng(tile_corner, ll);
            set_access(access);
            set_type(type);
            set_traffic_signal(traffic_signal);
            set_tagged_access(tagged_access);
            set_private_access(private_access);
            set_cash_only_toll(cash_only_toll);
        }

        public void set_latlng(PointLL tile_corner, PointLL ll)
        {
            lat_offset_ = 0;
            if (ll.lat() > tile_corner.lat())
            {
                var lat = Math.Round((ll.lat() - tile_corner.lat()) / 1e-7);
                lat_offset_ = (uint)(lat / 10);
                lat_offset7_ = (uint)(lat - lat_offset_ * 10);
            }

            lon_offset_ = 0;
            if (ll.lng() > tile_corner.lng())
            {
                var lon = Math.Round((ll.lng() - tile_corner.lng()) / 1e-7);
                lon_offset_ = (uint)(lon / 10);
                lon_offset7_ = (uint)(lon - lon_offset_ * 10);
            }
        }

        public void set_edge_index(uint edge_index)
        {
            if (edge_index > kMaxGraphId)
            {
                throw new Exception("NodeInfo: edge index exceeds max");
            }
            edge_index_ = edge_index;
        }

        public void set_edge_count(uint edge_count)
        {
            if (edge_count > kMaxEdgesPerNode)
            {
                edge_count_ = kMaxEdgesPerNode;
            }
            else
            {
                edge_count_ = edge_count;
            }
        }

        public void set_access(uint access)
        {
            if (access > kAllAccess)
            {
                access_ = (access & kAllAccess);
            }
            else
            {
                access_ = access;
            }
        }

        public void set_intersection(IntersectionType type)
        {
            intersection_ = (uint)type;
        }

        public void set_admin_index(uint admin_index)
        {
            if (admin_index > kMaxAdminsPerTile)
            {
                admin_index_ = kMaxAdminsPerTile;
            }
            else
            {
                admin_index_ = admin_index;
            }
        }

        public void set_timezone(uint tz_idx)
        {
            if (tz_idx > kMaxTimeZoneIdExt1)
            {
                throw new Exception("NodeInfo: timezone index exceeds max: " + tz_idx);
            }
            timezone_ = tz_idx & ((1 << 9) - 1);
            timezone_ext_1_ = (tz_idx & (1 << 9)) >> 9;
        }

        public void set_local_driveability(uint localidx, Traversability t)
        {
            if (localidx > kMaxLocalEdgeIndex)
            {
                return;
            }
            local_driveability_ = OverwriteBits(local_driveability_, (uint)t, localidx, 2);
        }

        public void set_density(uint density)
        {
            if (density > kMaxDensity)
            {
                density_ = kMaxDensity;
            }
            else
            {
                density_ = density;
            }
        }

        public void set_type(NodeType type)
        {
            type_ = (uint)type;
        }

        public void set_local_edge_count(uint n)
        {
            if (n > kMaxLocalEdgeIndex + 1)
            {
                local_edge_count_ = kMaxLocalEdgeIndex;
            }
            else if (n == 0)
            {
                throw new Exception("Node with 0 local edges found");
            }
            else
            {
                local_edge_count_ = n - 1;
            }
        }

        public void set_drive_on_right(bool rsd)
        {
            drive_on_right_ = rsd;
        }

        public void set_elevation(float elevation)
        {
            if (elevation < kNodeMinElevation)
            {
                elevation_ = 0;
            }
            else
            {
                uint elev = (uint)((elevation - kNodeMinElevation) / kNodeElevationPrecision);
                elevation_ = (elev > kNodeMaxStoredElevation) ? kNodeMaxStoredElevation : elev;
            }
        }

        public void set_tagged_access(bool tagged_access)
        {
            tagged_access_ = tagged_access;
        }

        public void set_mode_change(bool mc)
        {
            mode_change_ = mc;
        }

        public void set_named_intersection(bool named)
        {
            named_ = named;
        }

        public void set_traffic_signal(bool traffic_signal)
        {
            traffic_signal_ = traffic_signal;
        }

        public void set_stop_index(uint stop_index)
        {
            transition_index_ = stop_index;
        }

        public void set_heading(uint localidx, uint heading)
        {
            if (localidx > kMaxLocalEdgeIndex)
            {
                return;
            }
            uint hdg = (uint)Math.Round((heading % 360) * kHeadingShrinkFactor);
            headings_ |= hdg << (int)(localidx * 8);
        }

        public void set_connecting_wayid(ulong wayid)
        {
            if (wayid >> 63 != 0)
            {
                throw new Exception("Way ids larger than 63 bits are not allowed for transit connections");
            }
            headings_ = (uint)wayid;
        }

        public void set_connecting_point(PointLL p)
        {
            if (!p.InRange())
            {
                throw new Exception("Invalid coordinates are not allowed for transit connections");
            }
            headings_ = (uint)p | (1u << 63);
        }

        public JsonElement json(GraphTile tile)
        {
            var m = new Dictionary<string, object>
            {
                { "lon", (double)latlng(tile.header().base_ll()).Item1 },
                { "lat", (double)latlng(tile.header().base_ll()).Item2 },
                { "elevation", (double)elevation() },
                { "edge_count", (ulong)edge_count_ },
                { "access", access_json(access_) },
                { "tagged_access", tagged_access_ },
                { "intersection_type", intersection_.ToString() },
                { "administrative", admin_json(tile.admininfo(admin_index_), timezone_) },
                { "density", (ulong)density_ },
                { "local_edge_count", (ulong)(local_edge_count_ + 1) },
                { "drive_on_right", drive_on_right_ },
                { "mode_change", mode_change_ },
                { "private_access", private_access_ },
                { "traffic_signal", traffic_signal_ },
                { "type", type_.ToString() },
                { "transition count", (ulong)transition_count_ },
                { "named_intersection", named_ }
            };
            if (is_transit())
            {
                m.Add("stop_index", (ulong)stop_index());
            }
            return JsonDocument.Parse(JsonSerializer.Serialize(m)).RootElement;
        }

        private uint OverwriteBits(uint dst, uint src, uint pos, uint len)
        {
            uint shift = pos * len;
            uint mask = (((uint)1 << (int)len) - 1) << (int)shift;
            return (dst & ~mask) | (src << (int)shift);
        }
    }

    public struct PointLL
    {
        public double lat() { return 0; }
        public double lng() { return 0; }
        public bool InRange() { return true; }
    }

    public enum NodeType
    {
    }

    public enum IntersectionType
    {
    }

    public enum Traversability
    {
    }

    public class GraphTile
    {
        public GraphTileHeader header() { return new GraphTileHeader(); }
        public AdminInfo admininfo(uint index) { return new AdminInfo(); }
    }

    public class GraphTileHeader
    {
        public PointLL base_ll() { return new PointLL(); }
    }

    public class AdminInfo
    {
        public string country_iso() { return ""; }
        public string country_text() { return ""; }
        public string state_iso() { return ""; }
        public string state_text() { return ""; }
    }

    public static class DateTime
    {
        public static TimeZoneDB get_tz_db() { return new TimeZoneDB(); }
    }

    public class TimeZoneDB
    {
        public TimeZone from_index(uint index) { return new TimeZone(); }
    }

    public class TimeZone
    {
        public string name() { return ""; }
    }
}
