using System;
using System.Text.Json;
using valhalla.midgard;

namespace valhalla.baldr
{
    public class Location
    {
        public PointLL LatLng { get; }
        public StopType StopType { get; }
        public string Name { get; set; }
        public string Street { get; set; }
        public string DateTime { get; set; }
        public int? Heading { get; set; }
        public int HeadingTolerance { get; set; }
        public int NodeSnapTolerance { get; set; }
        public int StreetSideTolerance { get; set; }
        public int StreetSideMaxDistance { get; set; }
        public RoadClass StreetSideCutoff { get; set; }
        public int MinOutboundReach { get; set; }
        public int MinInboundReach { get; set; }
        public long Radius { get; set; }
        public PreferredSide PreferredSide { get; set; }
        public PointLL DisplayLatLng { get; set; }
        public int? PreferredLayer { get; set; }
        public SearchFilter SearchFilter { get; }

        public Location(PointLL latlng,
                        StopType stoptype,
                        int minOutboundReach = 0,
                        int minInboundReach = 0,
                        long radius = 0,
                        PreferredSide side = PreferredSide.Either,
                        RoadClass streetSideCutoff = RoadClass.ServiceOther,
                        SearchFilter searchFilter = null,
                        int? preferredLayer = null)
        {
            LatLng = latlng;
            StopType = stoptype;
            MinOutboundReach = minOutboundReach;
            MinInboundReach = minInboundReach;
            Radius = radius;
            PreferredSide = side;
            StreetSideCutoff = streetSideCutoff;
            SearchFilter = searchFilter ?? new SearchFilter();
            PreferredLayer = preferredLayer;
        }

        public bool Equals(Location other)
        {
            return LatLng.Equals(other.LatLng) &&
                   StopType == other.StopType &&
                   Name == other.Name &&
                   Street == other.Street &&
                   DateTime == other.DateTime &&
                   Heading == other.Heading &&
                   HeadingTolerance == other.HeadingTolerance &&
                   NodeSnapTolerance == other.NodeSnapTolerance &&
                   StreetSideTolerance == other.StreetSideTolerance &&
                   StreetSideMaxDistance == other.StreetSideMaxDistance &&
                   StreetSideCutoff == other.StreetSideCutoff &&
                   MinOutboundReach == other.MinOutboundReach &&
                   MinInboundReach == other.MinInboundReach &&
                   Radius == other.Radius &&
                   PreferredSide == other.PreferredSide &&
                   DisplayLatLng.Equals(other.DisplayLatLng) &&
                   PreferredLayer == other.PreferredLayer;
        }

        public JsonElement Json()
        {
            var locationMap = new
            {
                latlng = LatLng,
                stoptype = StopType,
                name = Name,
                street = Street,
                date_time = DateTime,
                heading = Heading,
                heading_tolerance = HeadingTolerance,
                node_snap_tolerance = NodeSnapTolerance,
                street_side_tolerance = StreetSideTolerance,
                street_side_max_distance = StreetSideMaxDistance,
                street_side_cutoff = StreetSideCutoff,
                min_outbound_reach = MinOutboundReach,
                min_inbound_reach = MinInboundReach,
                radius = Radius,
                preferred_side = PreferredSide,
                display_latlng = DisplayLatLng,
                preferred_layer = PreferredLayer
            };
            return JsonDocument.Parse(JsonSerializer.Serialize(locationMap)).RootElement;
        }
    }

    public class SearchFilter
    {
        public RoadClass MinRoadClass { get; }
        public RoadClass MaxRoadClass { get; }
        public bool ExcludeTunnel { get; }
        public bool ExcludeBridge { get; }
        public bool ExcludeToll { get; }
        public bool ExcludeRamp { get; }
        public bool ExcludeFerry { get; }
        public bool ExcludeClosures { get; }
        public float Level { get; }

        public SearchFilter(RoadClass minRoadClass = RoadClass.Motorway,
                            RoadClass maxRoadClass = RoadClass.ServiceOther,
                            bool excludeTunnel = false,
                            bool excludeBridge = false,
                            bool excludeToll = false,
                            bool excludeRamp = false,
                            bool excludeFerry = false,
                            bool excludeClosures = false,
                            float level = 0)
        {
            MinRoadClass = minRoadClass;
            MaxRoadClass = maxRoadClass;
            ExcludeTunnel = excludeTunnel;
            ExcludeBridge = excludeBridge;
            ExcludeToll = excludeToll;
            ExcludeRamp = excludeRamp;
            ExcludeFerry = excludeFerry;
            ExcludeClosures = excludeClosures;
            Level = level;
        }
    }

    public enum StopType
    {
        Break,
        Through,
        Via
    }

    public enum PreferredSide
    {
        Either,
        Same,
        Opposite
    }

    public enum RoadClass
    {
        Motorway,
        Trunk,
        Primary,
        Secondary,
        Tertiary,
        Unclassified,
        Residential,
        ServiceOther
    }
}
