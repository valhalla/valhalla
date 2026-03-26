# Twisty Roads Routing Preference

**Date:** 2026-03-26
**Status:** Draft

## Summary

Add a `use_twisty_roads` costing parameter that makes the router prefer (or avoid) roads with high curvature, weighted by speed to reflect the rider/driver experience. Available on motorcycle and auto costing models.

## Motivation

Motorcyclists and sports car drivers often want routes that prioritize winding roads over the fastest path. Valhalla already computes and stores a per-edge curvature value (0-15) on every `DirectedEdge`, but no costing model uses it. This feature exposes that data through a familiar `use_*` slider.

## API Surface

A single new costing parameter:

```json
{
  "costing": "motorcycle",
  "costing_options": {
    "motorcycle": {
      "use_twisty_roads": 0.8
    }
  }
}
```

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `use_twisty_roads` | float | 0.0 - 1.0 | 0.5 | 0.0 = avoid twisty roads, 0.5 = neutral, 1.0 = strongly prefer |

Available on: `motorcycle`, `auto`.

The routing demo (`demos/routing/index-internal.html`) will expose this as a slider in the motorcycle and auto costing panels.

## Design

### Speed-Weighted Perceived Twistiness

Pure geometric curvature doesn't capture the riding experience. A gentle highway curve at 120 km/h feels twistier than a tight residential corner at 20 km/h. The metric combines curvature with speed:

```
perceived_twistiness = geometric_curvature * (speed / reference_speed)
```

This also naturally adapts to live/predicted traffic — a congested twisty road correctly scores lower.

### Speed Floor

Without a speed floor, the algorithm would route through residential neighborhoods with sharp grid turns. A 50 km/h speed floor prevents this:

- Below 50 km/h: no twisty bonus applied, regardless of curvature
- The averaged curvature metric (computed across all shape points during tile build) already filters "one sharp turn then straight" patterns — a grid intersection surrounded by straight segments averages low curvature

The speed floor is a hardcoded constant, not a user-facing parameter. It exists as a quality filter to prevent degenerate results.

### Cost Factor Computation

Applied inside `EdgeCost()` for motorcycle and auto costing:

```
Constants:
  TWISTY_SPEED_FLOOR  = 50 km/h
  REFERENCE_SPEED     = 80 km/h
  MAX_TWISTY_FACTOR   = 0.75  (up to 75% cost reduction at use_twisty_roads=1.0)

Per edge:
  speed = tile->GetSpeed(edge, ...)

  if speed < TWISTY_SPEED_FLOOR:
      twisty_bonus = 0.0
  else:
      geometric_curvature = edge->curvature() / 15.0        // normalize 0-15 to 0.0-1.0
      speed_factor = speed / REFERENCE_SPEED                  // >1.0 for fast roads
      perceived_twistiness = clamp(geometric_curvature * speed_factor, 0.0, 1.0)

      // use_twisty_roads: 0.0-0.5 = penalty, 0.5 = neutral, 0.5-1.0 = bonus
      preference = (use_twisty_roads - 0.5) * 2.0            // maps to -1.0 to 1.0
      twisty_bonus = preference * perceived_twistiness * MAX_TWISTY_FACTOR

  factor = 1.0 - twisty_bonus   // <1.0 = cheaper (prefer), >1.0 = expensive (avoid)
  cost *= factor
```

### Example Behaviors

| use_twisty_roads | Curvature | Speed | Effect |
|-----------------|-----------|-------|--------|
| 1.0 | 15 (max) | 100 km/h | cost x 0.25 (75% reduction) |
| 1.0 | 15 (max) | 40 km/h | no effect (below speed floor) |
| 1.0 | 0 (straight) | any | no effect |
| 0.5 (default) | any | any | no effect |
| 0.0 | 15 (max) | 100 km/h | cost x 1.75 (75% penalty) |

## Files to Modify

| File | Change |
|------|--------|
| `proto/options.proto` | Add `use_twisty_roads` field to `Costing.Options` |
| `src/sif/motorcyclecost.cc` | Parse `use_twisty_roads`, apply twisty factor in `EdgeCost()` |
| `src/sif/autocost.cc` | Parse `use_twisty_roads`, apply twisty factor in `EdgeCost()` |
| `src/sif/dynamiccost.cc` | Parse `use_twisty_roads` in `ParseBaseCostOptions()` (shared across both models) |
| `valhalla/sif/dynamiccost.h` | Add `use_twisty_roads_` member to `DynamicCost` base class |
| `test/gurka/test_twisty_roads.cc` | New gurka integration test |
| `docs/docs/api/turn-by-turn/api-reference.md` | Document the new parameter |
| Routing demo HTML | Add slider for motorcycle/auto panels |

No tile format changes. No tile rebuild required. No new TaggedValues.

## Testing Strategy

### Gurka Integration Test (`test/gurka/test_twisty_roads.cc`)

ASCII map with two parallel paths between endpoints:
- **Straight path**: via straight primary roads
- **Twisty path**: via a curvy road (many shape points generating high curvature)

Test cases:

1. **Neutral (0.5)**: takes the shortest/fastest path (straight)
2. **Prefer (1.0)**: takes the curvy path
3. **Avoid (0.0)**: takes the straight path (confirms penalty direction)
4. **Speed floor**: curvy geometry on a low-speed residential road — twisty bonus must not apply
5. **Both costing models**: verify for motorcycle and auto

## Constraints

- **No tile format changes** — curvature already exists as a 4-bit field on DirectedEdge
- **No performance impact** — one multiply added to EdgeCost(), both inputs (curvature, speed) already read
- **Backward compatible** — default 0.5 produces identical routes to current behavior
