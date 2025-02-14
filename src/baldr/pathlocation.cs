using System;
using System.Collections.Generic;
using System.Text.Json;
using valhalla.midgard;

namespace valhalla.baldr
{
    public class PathLocation
    {
        public PointLL LatLng { get; }
        public LocationType Type { get; }
        public List<PathEdge> Edges { get; }
        public List<PathEdge> FilteredEdges { get; }

        public PathLocation(PointLL latlng, LocationType type)
        {
            LatLng = latlng;
            Type = type;
            Edges = new List<PathEdge>();
            FilteredEdges = new List<PathEdge>();
        }

        public JsonElement Json()
        {
            var pathLocationMap = new
            {
                latlng = LatLng,
                type = Type,
                edges = Edges,
                filtered_edges = FilteredEdges
            };
            return JsonDocument.Parse(JsonSerializer.Serialize(pathLocationMap)).RootElement;
        }
    }

    public class PathEdge
    {
        public GraphId Id { get; }
        public float PercentAlong { get; }
        public PointLL Projected { get; }
        public float Distance { get; }
        public bool IsForward { get; }
        public bool BeginNode { get; }
        public bool EndNode { get; }

        public PathEdge(GraphId id, float percentAlong, PointLL projected, float distance, bool isForward, bool beginNode, bool endNode)
        {
            Id = id;
            PercentAlong = percentAlong;
            Projected = projected;
            Distance = distance;
            IsForward = isForward;
            BeginNode = beginNode;
            EndNode = endNode;
        }
    }

    public enum LocationType
    {
        Break,
        Through,
        Via
    }
}
