using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;

namespace valhalla.baldr
{
    public class connectivity_map_t
    {
        private Dictionary<int, Dictionary<uint, int>> colors;
        private int transit_level;

        public connectivity_map_t(Dictionary<string, object> pt, GraphReader graph_reader)
        {
            colors = new Dictionary<int, Dictionary<uint, int>>();
            transit_level = TileHierarchy.GetTransitLevel().level;

            var tiles = graph_reader.GetTileSet();

            foreach (var t in tiles)
            {
                if (!colors.ContainsKey(t.level))
                {
                    colors[t.level] = new Dictionary<uint, int>();
                }
                colors[t.level][t.tileid] = 0;
            }

            foreach (var color in colors)
            {
                if (color.Key == transit_level)
                {
                    TileHierarchy.GetTransitLevel().tiles.ColorMap(color.Value, new Dictionary<uint, uint>());
                }
                else
                {
                    TileHierarchy.levels[color.Key].tiles.ColorMap(color.Value, color.Key == 2 ? new Dictionary<uint, uint>() : new Dictionary<uint, uint>());
                }
            }
        }

        public bool level_color_exists(uint level)
        {
            return colors.ContainsKey((int)level);
        }

        public int get_color(GraphId id)
        {
            if (colors.TryGetValue(id.level, out var levelColors) && levelColors.TryGetValue(id.tileid, out var color))
            {
                return color;
            }
            return 0;
        }

        public HashSet<int> get_colors(TileLevel hierarchy_level, PathLocation location, float radius)
        {
            var result = new HashSet<int>();
            if (!colors.TryGetValue(hierarchy_level.level, out var levelColors))
            {
                return result;
            }

            var edgeSets = new List<List<PathLocation.Edge>> { location.edges, location.filtered_edges };
            foreach (var edges in edgeSets)
            {
                foreach (var edge in edges)
                {
                    var ll = edge.projected;
                    var approximator = new DistanceApproximator<PointLL>(ll);
                    var latdeg = radius / Constants.kMetersPerDegreeLat;
                    var lngdeg = radius / approximator.MetersPerLngDegree(ll.lat);
                    var bbox = new AABB2<PointLL>(ll.lng - lngdeg, ll.lat - latdeg, ll.lng + lngdeg, ll.lat + latdeg);
                    var tilelist = hierarchy_level.tiles.TileList(bbox);
                    foreach (var id in tilelist)
                    {
                        if (levelColors.TryGetValue((uint)id, out var color))
                        {
                            result.Add(color);
                        }
                    }
                }
            }
            return result;
        }

        public string to_geojson(uint hierarchy_level)
        {
            if (hierarchy_level > TileHierarchy.GetTransitLevel().level)
            {
                throw new Exception("hierarchy level not found");
            }

            var tiles = hierarchy_level == transit_level ? TileHierarchy.GetTransitLevel().tiles : TileHierarchy.levels[(int)hierarchy_level].tiles;

            var regions = new Dictionary<int, HashSet<uint>>();
            if (colors.TryGetValue((int)hierarchy_level, out var levelColors))
            {
                foreach (var tile in levelColors)
                {
                    if (!regions.ContainsKey(tile.Value))
                    {
                        regions[tile.Value] = new HashSet<uint>();
                    }
                    regions[tile.Value].Add(tile.Key);
                }
            }

            var arities = new SortedDictionary<int, int>(Comparer<int>.Create((a, b) => b.CompareTo(a)));
            foreach (var region in regions)
            {
                arities[region.Value.Count] = region.Key;
            }

            var boundaries = new Dictionary<int, List<List<PointLL>>>();
            foreach (var arity in arities)
            {
                boundaries[arity.Value] = to_boundary(regions[arity.Value], tiles);
            }

            return to_feature_collection(boundaries, arities);
        }

        private List<List<PointLL>> to_boundary(HashSet<uint> region, Tiles tiles)
        {
            // Implement the logic to convert region to boundary
            return new List<List<PointLL>>();
        }

        private string to_feature_collection(Dictionary<int, List<List<PointLL>>> boundaries, SortedDictionary<int, int> arities)
        {
            var features = new List<Dictionary<string, object>>();
            var random = new Random(17);
            foreach (var arity in arities)
            {
                var color = $"#{random.Next(64, 192):X2}{random.Next(64, 192):X2}{random.Next(64, 192):X2}";
                features.Add(to_feature(boundaries[arity.Value], color));
            }

            var featureCollection = new Dictionary<string, object>
            {
                { "type", "FeatureCollection" },
                { "features", features }
            };

            return JsonSerializer.Serialize(featureCollection);
        }

        private Dictionary<string, object> to_feature(List<List<PointLL>> boundary, string color)
        {
            var geometry = new Dictionary<string, object>
            {
                { "type", "Polygon" },
                { "coordinates", boundary.Select(ring => ring.Select(coord => new List<double> { coord.lng, coord.lat }).ToList()).ToList() }
            };

            var properties = new Dictionary<string, object>
            {
                { "fill", color },
                { "stroke", "white" },
                { "stroke-width", 1 },
                { "fill-opacity", 0.8 }
            };

            return new Dictionary<string, object>
            {
                { "type", "Feature" },
                { "geometry", geometry },
                { "properties", properties }
            };
        }
    }
}
