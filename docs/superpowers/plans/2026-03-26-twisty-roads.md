# Twisty Roads Routing Preference — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `use_twisty_roads` costing parameter (0.0–1.0) to motorcycle and auto costing that makes the router prefer (or avoid) roads with high curvature, weighted by speed.

**Architecture:** Extends the existing `use_*` costing parameter pattern. A new protobuf field flows through JSON parsing into a `twisty_factor_` member on each costing class. In `EdgeCost()`, the existing `edge->curvature()` (0–15) is combined with the edge's speed to produce a perceived-twistiness score, which modulates the cost factor. A speed floor (50 km/h) prevents the bonus from applying on slow residential streets.

**Tech Stack:** C++20, Protobuf, CMake, GoogleTest (gurka framework)

**Spec:** `docs/superpowers/specs/2026-03-26-twisty-roads-design.md`

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `proto/options.proto` | Modify | Add `use_twisty_roads` field (number 97) to `Costing.Options` |
| `src/sif/motorcyclecost.cc` | Modify | Parse option, compute twisty factor, apply in `EdgeCost()` |
| `src/sif/autocost.cc` | Modify | Parse option, compute twisty factor, apply in `EdgeCost()` |
| `test/gurka/test_twisty_roads.cc` | Create | Gurka integration tests for twisty road preference |
| `docs/docs/api/turn-by-turn/api-reference.md` | Modify | Document new parameter |

**Design decision — no base class changes:** Since only two costing models use this parameter (motorcycle and auto), and each has its own `EdgeCost()` implementation with different factor-accumulation patterns, we keep the twisty factor as a local member in each class rather than adding to `DynamicCost`. This follows the pattern of `highway_factor_` and `toll_factor_` which are also per-model members.

---

### Task 1: Add protobuf field

**Files:**
- Modify: `proto/options.proto:354-359` (after `speed_penalty_factor`, before closing brace)

- [ ] **Step 1: Add `use_twisty_roads` field to `Costing.Options`**

In `proto/options.proto`, find the end of the `Options` message inside `Costing` (around line 358, after `multimodal_start_end_max_distance`). Add the new field at the next available number (97):

```protobuf
    oneof has_use_twisty_roads {
      float use_twisty_roads = 97;
    }
```

Insert this immediately before the closing `}` of the `Options` message (line 360).

- [ ] **Step 2: Rebuild to verify protobuf compiles**

Run:
```bash
cd build && cmake --build . -j$(nproc) --target valhalla-proto 2>&1 | tail -5
```

Expected: Build succeeds with no errors.

- [ ] **Step 3: Commit**

```bash
git add proto/options.proto
git commit -m "feat: add use_twisty_roads protobuf field to Costing.Options"
```

---

### Task 2: Add twisty factor to MotorcycleCost

**Files:**
- Modify: `src/sif/motorcyclecost.cc`

- [ ] **Step 1: Add constants and range**

At the top of `src/sif/motorcyclecost.cc`, after the existing `kDefaultUseTrails` constant (line 29), add:

```cpp
constexpr float kDefaultUseTwistyRoads = 0.5f; // Factor between 0 and 1. 0.5 = neutral
```

After the existing `kUseTrailsRange` constant (line 56), add:

```cpp
constexpr ranged_default_t<float> kUseTwistyRoadsRange{0, kDefaultUseTwistyRoads, 1.0f};
```

Add the twisty roads constants (after the range definition):

```cpp
// Twisty roads constants
constexpr float kTwistySpeedFloor = 50.0f;     // km/h - no twisty bonus below this speed
constexpr float kTwistyReferenceSpeed = 80.0f;  // km/h - normalizing speed for perceived twistiness
constexpr float kMaxTwistyFactor = 0.75f;        // max cost reduction at use_twisty_roads=1.0
```

- [ ] **Step 2: Add member variable and constructor initialization**

In the `MotorcycleCost` class definition (around line 265), find the member variable declarations. After `surface_factor_` (around line 290), add:

```cpp
  float twisty_factor_;       // Factor for twisty road preference
```

In the `MotorcycleCost` constructor (around line 297), after the `surface_factor_` computation block (around line 345), add:

```cpp
  // Twisty road preference - compute a factor from use_twisty_roads (0-1).
  // 0.5 = neutral (factor 0), 1.0 = prefer (negative factor), 0.0 = avoid (positive factor)
  float use_twisty_roads = costing_options.use_twisty_roads();
  twisty_factor_ = (use_twisty_roads - 0.5f) * 2.0f; // maps 0-1 to -1.0 to 1.0
```

- [ ] **Step 3: Apply twisty factor in EdgeCost()**

In `MotorcycleCost::EdgeCost()` (around line 402), find the line `factor *= EdgeFactor(edgeid);` (around line 443). Add the twisty factor computation **before** that line:

```cpp
  // Apply twisty road preference based on curvature and speed
  if (twisty_factor_ != 0.0f && edge_speed >= kTwistySpeedFloor) {
    float curvature = static_cast<float>(edge->curvature()) / 15.0f;
    float speed_weight = static_cast<float>(edge_speed) / kTwistyReferenceSpeed;
    float perceived_twistiness = std::min(curvature * speed_weight, 1.0f);
    factor *= 1.0f - twisty_factor_ * perceived_twistiness * kMaxTwistyFactor;
  }
```

Note: `edge_speed` is already computed earlier in the function (around line 407). Use `edge_speed` (the raw speed from `GetSpeed`), not `final_speed` (which is capped by `top_speed_`), since the perceived twistiness should reflect the road's actual speed, not the vehicle's speed limit.

- [ ] **Step 4: Add option parsing**

In `ParseMotorcycleCostOptions()` (around line 587), after the `kUseTrailsRange` line (line 601), add:

```cpp
  JSON_PBF_RANGED_DEFAULT(co, kUseTwistyRoadsRange, json, "/use_twisty_roads", use_twisty_roads, warnings);
```

- [ ] **Step 5: Build to verify compilation**

Run:
```bash
cd build && cmake --build . -j$(nproc) --target valhalla 2>&1 | tail -10
```

Expected: Build succeeds.

- [ ] **Step 6: Commit**

```bash
git add src/sif/motorcyclecost.cc
git commit -m "feat: add twisty road preference to motorcycle costing"
```

---

### Task 3: Add twisty factor to AutoCost

**Files:**
- Modify: `src/sif/autocost.cc`

- [ ] **Step 1: Add constants and range**

At the top of `src/sif/autocost.cc`, after the existing `kDefaultUseDistance` constant (line 36), add:

```cpp
constexpr float kDefaultUseTwistyRoads = 0.5f; // Factor between 0 and 1. 0.5 = neutral
```

After the existing `kUseDistanceRange` constant (line 76), add:

```cpp
constexpr ranged_default_t<float> kUseTwistyRoadsRange{0, kDefaultUseTwistyRoads, 1.0f};
```

Add the twisty roads constants (after the range definition):

```cpp
// Twisty roads constants
constexpr float kTwistySpeedFloor = 50.0f;     // km/h - no twisty bonus below this speed
constexpr float kTwistyReferenceSpeed = 80.0f;  // km/h - normalizing speed for perceived twistiness
constexpr float kMaxTwistyFactor = 0.75f;        // max cost reduction at use_twisty_roads=1.0
```

- [ ] **Step 2: Add member variable and constructor initialization**

In the `AutoCost` class definition (around line 340), find the member variable declarations. After `inv_distance_factor_` (around line 356), add:

```cpp
  float twisty_factor_;       // Factor for twisty road preference
```

In the `AutoCost` constructor (around line 367), after the `inv_distance_factor_` line (around line 399), add:

```cpp
  // Twisty road preference
  float use_twisty_roads = costing_options.use_twisty_roads();
  twisty_factor_ = (use_twisty_roads - 0.5f) * 2.0f; // maps 0-1 to -1.0 to 1.0
```

- [ ] **Step 3: Apply twisty factor in EdgeCost()**

In `AutoCost::EdgeCost()` (around line 495), find the line `factor *= EdgeFactor(edgeid);` (around line 556). Add the twisty factor computation **before** that line:

```cpp
  // Apply twisty road preference based on curvature and speed
  if (twisty_factor_ != 0.0f && edge_speed >= kTwistySpeedFloor) {
    float curvature = static_cast<float>(edge->curvature()) / 15.0f;
    float speed_weight = static_cast<float>(edge_speed) / kTwistyReferenceSpeed;
    float perceived_twistiness = std::min(curvature * speed_weight, 1.0f);
    factor *= 1.0f - twisty_factor_ * perceived_twistiness * kMaxTwistyFactor;
  }
```

Note: `edge_speed` is already computed earlier in the function (around line 501). Use `edge_speed` (the raw speed from `GetSpeed`), not `final_speed`.

- [ ] **Step 4: Add option parsing**

In `ParseAutoCostOptions()` (around line 705), after the `kUseTollsRange` line (line 719), add:

```cpp
  JSON_PBF_RANGED_DEFAULT(co, kUseTwistyRoadsRange, json, "/use_twisty_roads", use_twisty_roads, warnings);
```

- [ ] **Step 5: Build to verify compilation**

Run:
```bash
cd build && cmake --build . -j$(nproc) --target valhalla 2>&1 | tail -10
```

Expected: Build succeeds.

- [ ] **Step 6: Commit**

```bash
git add src/sif/autocost.cc
git commit -m "feat: add twisty road preference to auto costing"
```

---

### Task 4: Write gurka integration tests

**Files:**
- Create: `test/gurka/test_twisty_roads.cc`

- [ ] **Step 1: Write the test file**

Create `test/gurka/test_twisty_roads.cc`:

```cpp
#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// ASCII map with two paths between A and F:
// - Straight path: A -> B -> F (straight primary road)
// - Twisty path: A -> C -> d -> D -> e -> E -> ... -> J -> F (zigzag through offset nodes)
//
// The zigzag nodes create curvature during tile building.
// Grid size is 100m, so each row is ~100m apart and horizontal offsets create real curves.

const std::string kTwistyMap = R"(
    A---------B---------F
    |                   |
    |                   |
    C                   |
     d                  |
      D                 |
       e                |
        E               |
         f              |
          G             |
           g            |
            H           |
             h          |
              I---------J
  )";

class TwistyRoads : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const gurka::ways ways = {
        // Straight path — two long straight edges
        {"AB", {{"highway", "primary"}, {"maxspeed", "80"}}},
        {"BF", {{"highway", "primary"}, {"maxspeed", "80"}}},
        // Twisty path — zigzag through offset nodes, same road class and speed
        {"ACdDeDEefGgHhIJ",
         {{"highway", "primary"}, {"maxspeed", "80"}}},
        {"JF", {{"highway", "primary"}, {"maxspeed", "80"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(kTwistyMap, 100);
    map_ = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_twisty_roads",
                             {{"mjolnir.concurrency", "1"}});
  }

  static gurka::map map_;
};

gurka::map TwistyRoads::map_ = {};

// Test 1: Default (0.5) takes the shortest path (straight)
TEST_F(TwistyRoads, DefaultTakesStraightPath) {
  auto result = gurka::do_action(Options::route, map_, {"A", "F"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"AB", "BF"});
}

// Test 2: use_twisty_roads=1.0 takes the twisty path
TEST_F(TwistyRoads, PreferTwistyTakesCurvyPath) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "F"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "1.0"}});
  // Should NOT take the straight path — verify by checking path differs from straight
  const auto path = gurka::detail::get_paths(result).front();
  const std::vector<std::string> straight_path = {"AB", "BF"};
  EXPECT_NE(path, straight_path) << "Expected twisty path but got straight path";
}

// Test 3: use_twisty_roads=0.0 avoids twisty roads (takes straight)
TEST_F(TwistyRoads, AvoidTwistyTakesStraightPath) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "F"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "0.0"}});
  gurka::assert::raw::expect_path(result, {"AB", "BF"});
}

// Test 4: Works for auto costing too
TEST_F(TwistyRoads, AutoCostingPrefersTwisty) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "F"}, "auto",
                       {{"/costing_options/auto/use_twisty_roads", "1.0"}});
  // Should NOT take the straight path
  const auto path = gurka::detail::get_paths(result).front();
  const std::vector<std::string> straight_path = {"AB", "BF"};
  EXPECT_NE(path, straight_path) << "Expected twisty path but got straight path";
}

} // namespace

// Speed floor test — separate fixture with low-speed roads
namespace {

const std::string kSlowTwistyMap = R"(
    A---------B---------F
    |                   |
    C                   |
     d                  |
      D                 |
       e                |
        E               |
         f              |
          G             |
           g            |
            H-----------J
  )";

class TwistyRoadsSpeedFloor : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const gurka::ways ways = {
        // Straight path — residential speed
        {"AB", {{"highway", "residential"}, {"maxspeed", "30"}}},
        {"BF", {{"highway", "residential"}, {"maxspeed", "30"}}},
        // Twisty path — also residential speed (below 50 km/h floor)
        {"ACdDeDEefGgHJ",
         {{"highway", "residential"}, {"maxspeed", "30"}}},
        {"JF", {{"highway", "residential"}, {"maxspeed", "30"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(kSlowTwistyMap, 100);
    map_ = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_twisty_roads_slow",
                             {{"mjolnir.concurrency", "1"}});
  }

  static gurka::map map_;
};

gurka::map TwistyRoadsSpeedFloor::map_ = {};

// Test 5: Speed floor — twisty bonus should NOT apply on low-speed roads
TEST_F(TwistyRoadsSpeedFloor, SpeedFloorPreventsTwistyBonus) {
  // Even with max twisty preference, should take the straight (shorter) path
  // because all roads are below the 50 km/h speed floor
  auto result =
      gurka::do_action(Options::route, map_, {"A", "F"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "1.0"}});
  gurka::assert::raw::expect_path(result, {"AB", "BF"});
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Build and run the tests**

Run:
```bash
cd build && cmake --build . -j$(nproc) --target gurka_twisty_roads 2>&1 | tail -10
```

Expected: Build succeeds.

```bash
cd build && ./test/gurka/gurka_twisty_roads
```

Expected: All 5 tests pass.

**Important:** If `PreferTwistyTakesCurvyPath` fails (i.e., the zigzag path doesn't generate enough curvature to overcome the distance penalty), the map geometry needs adjustment. Try increasing the grid size to 200 or making the zigzag more extreme (wider horizontal offsets between alternating nodes).

- [ ] **Step 3: Commit**

```bash
git add test/gurka/test_twisty_roads.cc
git commit -m "test: add gurka integration tests for twisty road preference"
```

---

### Task 5: Document the new parameter

**Files:**
- Modify: `docs/docs/api/turn-by-turn/api-reference.md`

- [ ] **Step 1: Find the costing options section for auto and motorcycle**

Search for existing `use_highways` or `use_tolls` documentation in `docs/docs/api/turn-by-turn/api-reference.md`. These will be in tables describing costing options for auto and motorcycle.

- [ ] **Step 2: Add `use_twisty_roads` to auto costing options table**

In the auto costing options table, after the `use_tolls` row, add:

```markdown
| `use_twisty_roads` | A factor that allows preference or avoidance of roads with high curvature. The factor is a value from 0.0 to 1.0, where 0.0 avoids twisty roads, 0.5 is neutral (default), and 1.0 strongly prefers twisty roads. The preference is weighted by speed — faster curvy roads feel twistier. Roads below 50 km/h are not affected. |
```

- [ ] **Step 3: Add `use_twisty_roads` to motorcycle costing options table**

In the motorcycle costing options table, after the `use_tolls` row, add the same description:

```markdown
| `use_twisty_roads` | A factor that allows preference or avoidance of roads with high curvature. The factor is a value from 0.0 to 1.0, where 0.0 avoids twisty roads, 0.5 is neutral (default), and 1.0 strongly prefers twisty roads. The preference is weighted by speed — faster curvy roads feel twistier. Roads below 50 km/h are not affected. |
```

- [ ] **Step 4: Commit**

```bash
git add docs/docs/api/turn-by-turn/api-reference.md
git commit -m "docs: document use_twisty_roads costing parameter"
```

---

### Task 6: Run related tests to verify no regressions

- [ ] **Step 1: Run baseline motorcycle and auto tests**

Run the tests most likely to be affected by costing changes:

```bash
cd build && cmake --build . -j$(nproc) --target gurka_twisty_roads --target gurka_route --target gurka_shortest 2>&1 | tail -5
```

```bash
cd build && ./test/gurka/gurka_twisty_roads && ./test/gurka/gurka_route && ./test/gurka/gurka_shortest
```

Expected: All tests pass. Since the default `use_twisty_roads` is 0.5 (neutral, `twisty_factor_` = 0.0), no existing routes should change.

- [ ] **Step 2: Run additional related tests**

```bash
cd build && cmake --build . -j$(nproc) --target gurka_use_living_streets --target gurka_ferry_connections --target gurka_recost 2>&1 | tail -5
```

```bash
cd build && ./test/gurka/gurka_use_living_streets && ./test/gurka/gurka_ferry_connections && ./test/gurka/gurka_recost
```

Expected: All pass (no regression from our changes).

- [ ] **Step 3: Run format check**

```bash
cd /Users/anders/workspace/private/valhalla/.yolo/worktrees/valhalla-1/valhalla && ./scripts/format.sh
```

If formatting changes are made, commit them:

```bash
git add -u && git commit -m "style: apply clang-format"
```

---

### Task 7: Add slider to routing demo (follow-up)

The spec calls for a `use_twisty_roads` slider in the routing demo (`demos/routing/index-internal.html`). The demos live in a separate repository ([valhalla/demos](https://github.com/valhalla/demos)). If the demos are available locally (e.g., at `/Users/anders/workspace/private/valhalla-test/`), add a slider following the pattern of existing `use_*` sliders in the motorcycle and auto costing panels.

- [ ] **Step 1: Locate the demos directory**

Check if the demos repo is available:
```bash
ls /Users/anders/workspace/private/valhalla-test/demos/routing/index-internal.html 2>/dev/null && echo "Found" || echo "Not found — skip this task"
```

If not found, skip this task and note it as a follow-up for the demos repo.

- [ ] **Step 2: Add slider to motorcycle and auto panels**

Search for existing `use_highways` or `use_tolls` sliders in the HTML file and add a `use_twisty_roads` slider following the same pattern. The slider should:
- Label: "Prefer Twisty Roads"
- Range: 0.0 to 1.0, step 0.1
- Default: 0.5
- Map to `costing_options.<costing>.use_twisty_roads` in the request JSON
- Appear in both motorcycle and auto costing panels

- [ ] **Step 3: Commit**

```bash
git add demos/routing/index-internal.html
git commit -m "feat: add use_twisty_roads slider to routing demo"
```
