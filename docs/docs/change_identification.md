# Change identification

Valhalla exposes three different identifiers that answer three different questions about "did something change?". They live at different scopes and are computed independently — don't conflate them.

| Identifier | Scope | Width | Where it lives |
| :--------- | :---- | :---- | :------------- |
| `dataset_id` | global tileset (user-definable) | 64 bit | `GraphTileHeader::dataset_id_` |
| `tile_checksum` | a single tile | 48 bit | low bits of `GraphTileHeader::checksum_` |
| `build_id` | the whole tileset | 16 bit | high bits of `GraphTileHeader::checksum_` |

## `dataset_id` — which tileset?

A free-form integer identifying the *global tileset*. By default it is derived from the parsed OSM data — the maximum changeset id seen, falling back to the maximum OSM object (node/way) id if no changeset ids are present.

That default is just a convenience. Set `mjolnir.dataset_id` in the config to override it with **any** integer you want — a release number, a build counter, an epoch etc. Valhalla simply stamps the tile headers, it's the operator's choice how to interpret it.

It's the fully backward-compatible identification for Valhalla instances pre-dating [the `checksum_` PR](https://github.com/valhalla/valhalla/pull/6123).

It is surfaced as `osm_changeset` in route responses and in the verbose [`/status`](api/status/api-reference.md) output.

## `tile_checksum` — did this tile's data change?

An MD5 hash of a single tile's data portion (edges, nodes, edge info, elevation, …), folded down to 48 bits. It is computed while the tile is streamed to disk and stamped into the header.

## `build_id` — did the tileset change?

A tileset-wide fingerprint: the sum of every tile's `tile_checksum`, folded to 16 bits, then stamped into the high bits of *every* tile's `checksum_`. Because it is a sum, it is order-independent, and because it is stamped uniformly, every tile in a tileset carries the *same* `build_id`, and _any_ tile data/OSM change will result will very likely result in a different `tile_checksum`.

A `build_id` of `0` together with a `tile_checksum` of `0` means the tile was built by an older Valhalla that predates the checksum field — update the tile-building instance.
