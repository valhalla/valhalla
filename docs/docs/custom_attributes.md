# Custom Attributes 

Valhalla allows custom attributes to be set per edge via a separate extract referred to as the custom attributes extract. Storage of and access to these attributes works similar to live traffic: every tile has a corresponding custom attributes tile in a separate tar archive (the extract) whose layout is a header followed by an array of per-edge attributes. The index position of an edge in a tile is also the index of the edge in the custom attributes tile. Multiple named attributes can be stored per edge and independently used in costing at request time. Use cases include attributes that Valhalla does not support, either because they are not derived from OSM data, or because they are too specific or subjective to be supported in the public project. Examples are walkability scores, cycling safety scores derived from accident reports, or routes popular with fleet drivers.

## How it works

The extract is a tar archive (`custom_attributes.tar`) with the same directory structure as `valhalla_tiles.tar`. For each routing tile `2/001/234/567.gph` there is a corresponding sidecar tile `2/001/234/567.gph`. A `.gph` file is a compact binary:

```
[CustomAttributesTileHeader (see `custom_attributes_tile.h`)]
[attr[0][0], attr[0][1], ..., attr[0][num_attributes-1]] edge 0
[attr[1][0], attr[1][1], ..., attr[1][num_attributes-1]] edge 1
...
[attr[edge_count-1][0], ..., attr[edge_count-1][num_attributes-1]]
```

The data is row-major by edge: all attributes for edge 0, then all attributes for edge 1, etc. Currently, each 8-bit unsigned integer stores a single edge attribute, but the format can be extended to support more widths and types. The local edge index is the same as `GraphId::id()`.

The extract also contains a single `attributes.json` file at its root:

```json
[{"name": "scenic", "type": "uint8"}, {"name": "walkability", "type": "uint8"}]
```

This JSON array declares the attribute names in column order (column 0 = first name, column 1 = second name, etc.).

`GraphReader` loads the extract at startup (read-only mmap). The archive is held open so an external process can update it while the service runs, analogous to how live traffic works. 

## Building the extract

`scripts/build_custom_attributes_tar.py` creates an initial extract from an existing `valhalla_tiles.tar`. All edges get default values. The `--attribute-names` flag is required and determines both the number of attributes per edge and the names written to `attributes.json` inside the tar.

```bash
python3 scripts/valhalla_build_custom_attributes_extract \
    --config         valhalla.json \
    --attribute-defs '[{"name": "scenic", "type": "uint8", "scale": 0.01}]' \
    --default-values 0.0
```

After writing the initial extract, replace individual entries in-place to populate real values. The in-memory format is simple enough to update with any language that supports `struct.pack` or equivalent. The extract is memory mapped read-write by the update process, read-only by the routing service — the same arrangement as `traffic.tar`.

**Important:** Always regenerate or update the extract from the same tile set it was built against. Edge indices are stable within a tileset version but will shift if tiles are rebuilt from new OSM data. There is no cross-check between the sidecar and the main tileset at load time; a mismatched sidecar silently applies values to the wrong edges.

## Filling the extract from OSM tags

`valhalla_add_custom_attributes` writes values derived from OSM way tags into an existing sidecar. It reads the graph and the sidecar from the config (`mjolnir.tile_extract` or `mjolnir.tile_dir`, and `mjolnir.custom_attributes_extract`), then streams the ways of the PBF the graph was built from.

```bash
valhalla_add_custom_attributes -c valhalla.json \
    --tag-keys surface,smoothness \
    --lua surface_quality.lua \
    planet.osm.pbf
```

Ways that have none of the `--tag-keys` are skipped before Lua runs. Each value is written to every edge of that way. Names missing from `attributes.json` are reported once and ignored, and unused attributes are not touched.

```lua
local smoothness = { excellent = 0.0, good = 0.1, intermediate = 0.3, bad = 0.6, very_bad = 1.0 }

function custom_attributes(tags)
  local v = smoothness[tags["smoothness"]]
  if v then
    return { surface_quality = v }
  end
  if tags["surface"] == "gravel" then
    -- return a single table that will be applied to edges in both directions
    -- OR return two tables, one for forward edges, one for the backward ones!
    return { surface_quality = 0.5 }  --, { surface_quality = 0.5 }
  end
  return nil
end
```

## Configuration

Add `custom_attributes_extract` to the `mjolnir` section of your config. 

```json
{
  "mjolnir": {
    "tile_extract":              "/data/valhalla/tiles.tar",
    "traffic_extract":           "/data/valhalla/traffic.tar",
    "custom_attributes_extract": "/data/custom_attributes.tar"
  }
}
```

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
factor += weighted sum of (attr_i * weight_i) for all i where weight_i ≠ 0
```

`factor` is then multiplied by the base edge cost, along with the costing's other factors. An attribute value of `1.0` with `weight=1.0` adds `1.0` to `factor`. A higher factor makes an edge more costly.

## Custom attributes in `trace_attributes`

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

Enable it by adding `"edge.custom_attribute": true` to the `filters.attributes` list. Keys are the attribute names from `attributes.json`.

## Custom attributes in `tile`

The `edges` and `shortcuts` layers of the [tile endpoint](api/tile.md) carry one property per attribute and direction, named `custom:<name>:fwd` and `custom:<name>:bwd`, e.g. `custom:scenic:fwd`. Enable them with `"edge.custom_attribute"` in `filters.attributes` or with `verbose`.
