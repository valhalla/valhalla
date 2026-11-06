# Custom Attributes 

Valhalla allows custom attributes to be set per edge via a separate tar archive referred to as the custom attributes extract. Storage of and access to these attributes works similar to live traffic: every tile has a corresponding custom attributes tile in a separate tar archive (the extract) whose layout is a header followed by an array of per-edge attributes. The index position of an edge in a tile is also the index of the edge in the custom attributes tile. Multiple named attributes can be stored per edge and independently used in costing at request time. Use cases include attributes that Valhalla does not support, either because they are not derived from OSM data, or because they are too specific or subjective to be supported in the public project. Examples are walkability scores, cycling safety scores derived from accident reports, or routes popular with fleet drivers.

## How it works

The sidecar is a tar archive (`custom_attributes.tar`) with the same directory structure as `valhalla_tiles.tar`. For each routing tile `2/001/234/567.gph` there is a corresponding sidecar tile `2/001/234/567.cab`. A `.cab` file is a compact binary:

```
[CustomAttributesTileHeader (see `custom_attributes_tile.h`)]
[attr[0][0], attr[0][1], ..., attr[0][num_attributes-1]] edge 0
[attr[1][0], attr[1][1], ..., attr[1][num_attributes-1]] edge 1
...
[attr[edge_count-1][0], ..., attr[edge_count-1][num_attributes-1]]
```

The data is row-major by edge: all attributes for edge 0, then all attributes for edge 1, etc. Currently, each 8-bit unsigned integer stores a single edge attribute, but the format can be extended to support more widths and types. The local edge index is the same as `GraphId::id()`.

The tar also contains a single `attributes.json` file at its root:

```json
[{"name": "scenic", "type": "uint8"}, {"name": "walkability", "type": "uint8"}]
```

This JSON array declares the attribute names in column order (column 0 = first name, column 1 = second name, etc.).

`GraphReader` loads the archive at startup (read-only mmap). The archive is held open so an external process can update it while the service runs, analogous to how live traffic works. 

## Building the sidecar tar

`scripts/build_custom_attributes_tar.py` creates an initial sidecar from an existing `valhalla_tiles.tar`. All edges get default values. The `--attribute-names` flag is required and determines both the number of attributes per edge and the names written to `attributes.json` inside the tar.

```bash
python3 scripts/build_custom_attributes_tar.py \
    --config         valhalla.json \
    --attribute-names scenic,surface_quality \
    --default-values 0.0,0.0
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-c`, `--config` | — | Valhalla config JSON. Reads tiles from `mjolnir.tile_extract`, writes the sidecar to `mjolnir.custom_attributes_extract` |
| `-i`, `--inline-config` | `{}` | Inline JSON config, overrides `--config` if present. One of the two is required |
| `--attribute-names` | required | Comma-separated names written to `attributes.json` in the tar. The number of names determines `num_attributes` per edge. |
| `--default-values` | `0.0` | Comma-separated default floats, one per attribute. Fewer values than the name count repeats the last value. |
| `--random` | off | Fill each slot with a random value in `[0.0, --random-max)` instead of the default |
| `--random-max` | `1.0` | Upper bound (exclusive) for random values when `--random` is set |

The script reads only the `GraphTileHeader` from each tile to determine `directededgecount`. It does **not** modify the main tile tar.

After writing the initial archive, replace individual `.cab` entries in-place to populate real values. The in-memory format is simple enough to update with any language that supports `struct.pack` or equivalent. The sidecar is mmap'd read-write by the update process, read-only by the routing service — the same arrangement as `traffic.tar`.

**Important:** Always regenerate or update the sidecar from the same tile set it was built against. Edge indices are stable within a tileset version but will shift if tiles are rebuilt from new OSM data. There is no cross-check between the sidecar and the main tileset at load time; a mismatched sidecar silently applies values to the wrong edges.

## Filling the sidecar from OSM tags

`valhalla_add_custom_attributes` writes values derived from OSM way tags into an existing sidecar. It reads the graph and the sidecar from the config (`mjolnir.tile_extract` or `mjolnir.tile_dir`, and `mjolnir.custom_attributes_extract`), then streams the ways of the PBF the graph was built from.

```bash
valhalla_add_custom_attributes -c valhalla.json \
    --tag-keys surface,smoothness \
    --lua surface_quality.lua \
    planet.osm.pbf
```

Ways that have none of the `--tag-keys` are skipped before Lua runs. The tags of the remaining ways are passed to `custom_attributes(tags)`, which returns a table of attribute name → number. Each value is written to every edge of that way. Names missing from `attributes.json` are reported once and ignored, and attributes the script does not return keep their current value.

```lua
local smoothness = { excellent = 0.0, good = 0.1, intermediate = 0.3, bad = 0.6, very_bad = 1.0 }

function custom_attributes(tags)
  local v = smoothness[tags["smoothness"]]
  if v then
    return { surface_quality = v }
  end
  if tags["surface"] == "gravel" then
    return { surface_quality = 0.5 }
  end
  return nil
end
```

Way edges come from `collect_way_edges` (`valhalla/mjolnir/way_edges_processor.h`), so only edges with forward auto access get values. Shortcut edges are not written.

## Configuration

Add only `custom_attributes_extract` to the `mjolnir` section of your config. Attribute names come from `attributes.json` inside the tar — no separate config entry is needed.

```json
{
  "mjolnir": {
    "tile_extract":              "/data/valhalla/tiles.tar",
    "traffic_extract":           "/data/valhalla/traffic.tar",
    "custom_attributes_extract": "/data/custom_attributes.tar"
  }
}
```

This key is optional — if the file does not exist or the key is omitted, routing proceeds normally with no custom attribute effect and no error.

## Using custom attributes in a route request

Set `use_custom_attributes` in the costing options as a JSON object mapping attribute names to weights. Valid weight range: `[-1.0, 1.0]`. Default: empty (disabled).

```json
{
  "locations": [
    {"lat": 52.5, "lon": 13.4},
    {"lat": 52.6, "lon": 13.5}
  ],
  "costing": "auto",
  "costing_options": {
    "auto": {
      "use_custom_attributes": {
        "scenic": 0.8,
        "surface_quality": 0.3
      }
    }
  }
}
```

Only names declared in `attributes.json` inside the tar are recognized. Unknown names are silently ignored. Omitted names default to weight `0.0` (no effect). The request fails gracefully if no sidecar is loaded — the weights are simply never applied.

Supported costing models: `auto`, `bus`, `taxi`, `truck`, `motorcycle`, `motor_scooter`, `bicycle`, `pedestrian`.

### Cost semantics

The custom attributes contribute additively to the edge cost factor:

```
factor += Σ (attr_i * weight_i)   for all i where weight_i ≠ 0
```

`factor` is then multiplied by the base edge traversal time. A `cab_value` of `1.0` with `weight=1.0` adds `1.0` to `factor` — equivalent to the maximum highway avoidance bias. Negative weights reduce the factor and can make an edge more attractive.

The factor penalty is proportional to edge traversal time: a slow 10 km/h urban street receives a larger absolute cost increase than a fast 100 km/h motorway for the same attribute value. This is intentional for penalizing time-weighted exposure (e.g. noise, pollution), but may not suit all use cases.

## Inspecting custom attribute values via `trace_attributes`

All per-edge attribute values for each matched edge are available in `trace_attributes` responses as a `custom_attributes` map:

```json
{
  "shape": "...",
  "edges": [
    {
      "custom_attributes": {
        "scenic": 0.42,
        "surface_quality": 0.87
      },
      ...
    }
  ]
}
```

Enable it by adding `"edge.custom_attribute": true` to the `filters.attributes` list. The object is omitted when no sidecar is loaded. Keys are the attribute names from `attributes.json`; attributes beyond the declared names are keyed by their numeric index string (`"0"`, `"1"`, ...).

`use_custom_attributes` in the costing options does **not** affect the values returned here — `custom_attributes` always contains the raw `.cab` floats, independent of the routing weights.

## Binary format reference

```
Offset              Size              Type        Field
0                   8                 uint64_t    tile_id (GraphId of the tile)
8                   8                 uint64_t    last_update (seconds since epoch)
16                  2                 uint16_t    num_attributes (bits 0-15 of a uint64_t)
18                  6                             spare (bits 16-63, 0)
24                  4                 uint32_t    directed_edge_count
28                  4                 uint32_t    custom_attributes_tile_version (1)
32                  4*N               float       attr[edge=0][attr=0..N-1]
32+4*N              4*N               float       attr[edge=1][attr=0..N-1]
...
32+(edge_count-1)*N*4 4*N             float       attr[edge=edge_count-1][attr=0..N-1]
```

Total file size must equal `sizeof(CustomAttributesTileHeader) + edge_count * num_attributes * sizeof(float)` exactly. A mismatch causes the tile to be skipped with a `LOG_ERROR` at startup; routing for that tile proceeds without custom attributes.

There is currently no format version field. Do not change the layout without updating both the C++ reader (`valhalla/baldr/custom_attributes_tile.h`) and the Python build script simultaneously.
