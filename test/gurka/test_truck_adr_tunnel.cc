#include "gurka.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

using namespace valhalla;

namespace {

// Expects the route to fail with "no path could be found for input" (442).
void expect_no_path(gurka::map& map,
                    const std::vector<std::string>& waypoints,
                    const std::unordered_map<std::string, std::string>& options) {
  try {
    gurka::do_action(Options::route, map, waypoints, "truck", options);
    FAIL() << "Expected no path to be found";
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  }
}

std::unordered_map<std::string, std::string> hazmat_options(const std::string& adr_tunnel_code) {
  std::unordered_map<std::string, std::string> options{{"/costing_options/truck/hazmat", "1"}};
  if (!adr_tunnel_code.empty()) {
    options["/costing_options/truck/adr_tunnel_code"] = adr_tunnel_code;
  }
  return options;
}

// The ADR 8.6.4 decision matrix under the conservative (worst-case) reading:
// quantity-conditional (B1000C, C5000D) and tank/bulk-conditional (B/D, B/E,
// C/D, C/E, D/E) clauses are assumed to apply, so passage is forbidden
// through a tunnel of category cat iff cat >= first_letter(code).
//
// This table is the shared truth table with the sn-adr Rust crate: it must
// stay cell-for-cell identical to fork/adr_matrix_test_vectors.json
// (generated from SPEC-ADR-COSTING.md §2.4/§2.5, which cites ADR 8.6.4,
// https://unece.org/transport/dangerous-goods).
struct AdrVector {
  // request spelling of the code; "" = hazmat load with no declared code
  std::string code;
  // passage allowed through tunnel categories A, B, C, D, E
  bool allowed[5];
};

// clang-format off
const std::vector<AdrVector> kAdrMatrix = {
    // code           A      B      C      D      E
    {"(—)",         {true,  true,  true,  true,  true}},
    {"B",           {true,  false, false, false, false}},
    {"B1000C",      {true,  false, false, false, false}},
    {"B/D",         {true,  false, false, false, false}},
    {"B/E",         {true,  false, false, false, false}},
    {"C",           {true,  true,  false, false, false}},
    {"C5000D",      {true,  true,  false, false, false}},
    {"C/D",         {true,  true,  false, false, false}},
    {"C/E",         {true,  true,  false, false, false}},
    {"D",           {true,  true,  true,  false, false}},
    {"D/E",         {true,  true,  true,  false, false}},
    {"E",           {true,  true,  true,  true,  false}},
    // hazmat load, no code declared: conservatively treated as code "B"
    {"",            {true,  false, false, false, false}},
};
// clang-format on

} // namespace

// Five disconnected corridors, one per explicit ADR tunnel category. The
// middle of each corridor is a tunnel tagged hazmat:adr_tunnel_cat=A..E.
class TruckAdrMatrixTest : public ::testing::Test {
protected:
  static gurka::map map;
  // {origin, destination, first way, tunnel way} per category A..E
  static constexpr std::array<std::array<const char*, 4>, 5> kCorridors = {{
      {"A", "C", "AB", "BC"},
      {"D", "F", "DE", "EF"},
      {"G", "I", "GH", "HI"},
      {"J", "L", "JK", "KL"},
      {"M", "O", "MN", "NO"},
  }};

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      A----B----C
      D----E----F
      G----H----I
      J----K----L
      M----N----O
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "A"}}},
        {"DE", {{"highway", "residential"}}},
        {"EF", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "B"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "C"}}},
        {"JK", {{"highway", "residential"}}},
        {"KL", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "D"}}},
        {"MN", {{"highway", "residential"}}},
        {"NO", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "E"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_adr_matrix");
  }
};

gurka::map TruckAdrMatrixTest::map = {};
constexpr std::array<std::array<const char*, 4>, 5> TruckAdrMatrixTest::kCorridors;

// The full shared truth table: 13 profiles x 5 tunnel categories.
TEST_F(TruckAdrMatrixTest, MatrixHazmat) {
  for (const auto& vector : kAdrMatrix) {
    for (size_t cat = 0; cat < kCorridors.size(); ++cat) {
      const auto& corridor = kCorridors[cat];
      const auto options = hazmat_options(vector.code);
      SCOPED_TRACE("code '" + vector.code + "' through category tunnel " + corridor[3]);
      if (vector.allowed[cat]) {
        auto route =
            gurka::do_action(Options::route, map, {corridor[0], corridor[1]}, "truck", options);
        gurka::assert::raw::expect_path(route, {corridor[2], corridor[3]});
      } else {
        expect_no_path(map, {corridor[0], corridor[1]}, options);
      }
    }
  }
}

// Without hazmat, tunnel categories are ignored entirely, even when a
// (contradictory) code is supplied.
TEST_F(TruckAdrMatrixTest, NonHazmatIgnoresCategories) {
  for (const auto* code : {"", "B", "E"}) {
    for (const auto& corridor : kCorridors) {
      std::unordered_map<std::string, std::string> options;
      if (*code) {
        options["/costing_options/truck/adr_tunnel_code"] = code;
      }
      auto route =
          gurka::do_action(Options::route, map, {corridor[0], corridor[1]}, "truck", options);
      gurka::assert::raw::expect_path(route, {corridor[2], corridor[3]});
    }
  }
}

// An unrecognised code is treated conservatively as code "B": blocked from
// the category B tunnel, allowed through category A.
TEST_F(TruckAdrMatrixTest, UnrecognisedCodeIsConservative) {
  const auto options = hazmat_options("B-D");
  expect_no_path(map, {"D", "F"}, options);
  auto route = gurka::do_action(Options::route, map, {"A", "C"}, "truck", options);
  gurka::assert::raw::expect_path(route, {"AB", "BC"});
}

// Tag extraction tiers of SPEC-ADR-COSTING §3 / the OSM hazmat scheme
// (https://wiki.openstreetmap.org/wiki/Key:hazmat).
class TruckAdrInferenceTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      A----B----C
      D----E----F
      G----H----I
      J----K----L
      M----N----O
    )";

    const gurka::ways ways = {
        // exclusion inference: highest excluded letter wins (E over B)
        {"AB", {{"highway", "residential"}}},
        {"BC",
         {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:B", "no"}, {"hazmat:E", "no"}}},
        // hazmat exclusion off a tunnel: same inference (ADR corridors are
        // tagged on approach ways too), category C
        {"DE", {{"highway", "residential"}}},
        {"EF", {{"highway", "residential"}, {"hazmat:C", "no"}}},
        // hazmat:X=yes is not an exclusion: no category, no ban
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:C", "yes"}}},
        // malformed explicit value falls through to exclusion inference
        {"JK", {{"highway", "residential"}}},
        {"KL",
         {{"highway", "residential"},
          {"tunnel", "yes"},
          {"hazmat:adr_tunnel_cat", "category-c"},
          {"hazmat:D", "no"}}},
        // plain tunnel with no hazmat tagging: unset category
        {"MN", {{"highway", "residential"}}},
        {"NO", {{"highway", "residential"}, {"tunnel", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_adr_inference");
  }
};

gurka::map TruckAdrInferenceTest::map = {};

TEST_F(TruckAdrInferenceTest, ExclusionInferenceHighestLetterWins) {
  // tunnel=yes + hazmat:B=no + hazmat:E=no => category E: only a code E load
  // (and every lower threshold) is blocked, an unrestricted "(—)" load passes
  expect_no_path(map, {"A", "C"}, hazmat_options("E"));
  expect_no_path(map, {"A", "C"}, hazmat_options("")); // no code -> treated as B
  auto route = gurka::do_action(Options::route, map, {"A", "C"}, "truck", hazmat_options("(—)"));
  gurka::assert::raw::expect_path(route, {"AB", "BC"});
}

TEST_F(TruckAdrInferenceTest, HazmatExclusionOffTunnelDerivesCategory) {
  // hazmat:C=no without tunnel=yes derives category C all the same: signed
  // ADR restrictions are tagged on whole corridors (approach ways included),
  // so the inference is not limited to tunnel=yes ways. A code-C load is
  // blocked, a code-D load and an explicitly unrestricted load pass, and a
  // hazmat load with no declared code stays conservatively excluded.
  expect_no_path(map, {"D", "F"}, hazmat_options("C"));
  expect_no_path(map, {"D", "F"}, hazmat_options(""));
  auto route_d = gurka::do_action(Options::route, map, {"D", "F"}, "truck", hazmat_options("D"));
  gurka::assert::raw::expect_path(route_d, {"DE", "EF"});
  auto route_unrestricted =
      gurka::do_action(Options::route, map, {"D", "F"}, "truck", hazmat_options("(—)"));
  gurka::assert::raw::expect_path(route_unrestricted, {"DE", "EF"});
  auto route = gurka::do_action(Options::route, map, {"D", "F"}, "truck");
  gurka::assert::raw::expect_path(route, {"DE", "EF"});
}

TEST_F(TruckAdrInferenceTest, HazmatYesIsNotAnExclusion) {
  // tunnel=yes + hazmat:C=yes derives no category and no ban
  auto route = gurka::do_action(Options::route, map, {"G", "I"}, "truck", hazmat_options("C"));
  gurka::assert::raw::expect_path(route, {"GH", "HI"});
}

TEST_F(TruckAdrInferenceTest, MalformedExplicitValueFallsThrough) {
  // hazmat:adr_tunnel_cat=category-c is unparseable, so the category comes
  // from the hazmat:D=no exclusion: blocked at threshold D, open at E, hence
  // exactly category D
  expect_no_path(map, {"J", "L"}, hazmat_options("D/E"));
  auto route = gurka::do_action(Options::route, map, {"J", "L"}, "truck", hazmat_options("E"));
  gurka::assert::raw::expect_path(route, {"JK", "KL"});
}

TEST_F(TruckAdrInferenceTest, UncategorisedTunnelUnaffected) {
  auto route = gurka::do_action(Options::route, map, {"M", "O"}, "truck", hazmat_options("B"));
  gurka::assert::raw::expect_path(route, {"MN", "NO"});
}

// A category C tunnel with a height-limited bypass: ADR restrictions and
// dimensional gates compose.
class TruckAdrDetourTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      A----B----C----D
           |    |
           E----F
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}, {"tunnel", "yes"}, {"hazmat:adr_tunnel_cat", "C"}}},
        {"CD", {{"highway", "residential"}}},
        {"BE", {{"highway", "residential"}}},
        {"EF", {{"highway", "residential"}, {"maxheight", "4.2"}}},
        {"FC", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_adr_detour");
  }
};

gurka::map TruckAdrDetourTest::map = {};

TEST_F(TruckAdrDetourTest, NoCodeUsesTunnel) {
  auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck");
  gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});
}

TEST_F(TruckAdrDetourTest, CodeCDetoursAroundCategoryC) {
  auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck", hazmat_options("C"));
  gurka::assert::raw::expect_path(route, {"AB", "BE", "EF", "FC", "CD"});
}

TEST_F(TruckAdrDetourTest, CodeDUnaffectedByCategoryC) {
  auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck", hazmat_options("D"));
  gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});
}

TEST_F(TruckAdrDetourTest, AdrAndHeightGatesCompose) {
  // the tunnel is barred by ADR and the bypass by maxheight=4.2: no path
  auto options = hazmat_options("C");
  options["/costing_options/truck/height"] = "4.5";
  expect_no_path(map, {"A", "D"}, options);
}

TEST_F(TruckAdrDetourTest, HeightAloneKeepsTunnelOpen) {
  // without hazmat the ADR category is ignored; only the bypass is
  // height-limited, so the tall truck routes through the tunnel
  auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck",
                                {{"/costing_options/truck/height", "4.5"}});
  gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});
}

// Real-world corridor tagging, mirroring the Beneluxtunnel (Rotterdam, OSM
// ways 29245400/34886842) and the Botlektunnel style named in the OSM hazmat
// scheme: the signed ADR restriction is tagged per-code on the approach ways
// (no tunnel=yes, no explicit category key) as well as on the tunnel way
// itself. Every way of the corridor must derive the same category or the
// blanket hazmat ban on the approaches blanket-detours every code alike.
class TruckAdrCorridorTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // Every restricted edge sits BEHIND an unrestricted approach edge:
    // costing (blanket hazmat restriction and ADR category alike) binds in
    // Allowed()/AllowedReverse() when the search EXPANDS onto an edge, while
    // origin/destination edges are correlated by loki's plain access filter.
    // A single-edge corridor would make every route trivial and bypass the
    // very checks under test.
    const std::string ascii_map = R"(
      A----B----C----D
      E----F----K
      G----H----L
      I----J----M
    )";

    const gurka::ways ways = {
        // the Beneluxtunnel corridor: approaches AB/CD carry the per-code
        // tags with no tunnel and no explicit category; the tunnel way BC
        // carries the identical tags plus tunnel=yes and the explicit
        // category, exactly as the real ways do. Everything derives/reads
        // category C.
        {"AB",
         {{"highway", "residential"},
          {"hazmat:B", "no"},
          {"hazmat:C", "no"},
          {"hazmat:D", "yes"},
          {"hazmat:E", "yes"}}},
        {"BC",
         {{"highway", "residential"},
          {"tunnel", "yes"},
          {"hazmat:adr_tunnel_cat", "C"},
          {"hazmat:B", "no"},
          {"hazmat:C", "no"},
          {"hazmat:D", "yes"},
          {"hazmat:E", "yes"}}},
        {"CD",
         {{"highway", "residential"},
          {"hazmat:B", "no"},
          {"hazmat:C", "no"},
          {"hazmat:D", "yes"},
          {"hazmat:E", "yes"}}},
        // blanket-plus-allowance style: hazmat=no refined by hazmat:E=yes is
        // a graded restriction - category D (the highest letter not
        // explicitly allowed), with the crude blanket ban suppressed
        {"EF", {{"highway", "residential"}}},
        {"FK", {{"highway", "residential"}, {"hazmat", "no"}, {"hazmat:E", "yes"}}},
        // plain hazmat=no with no per-code sub-keys: no category, binary ban
        {"GH", {{"highway", "residential"}}},
        {"HL", {{"highway", "residential"}, {"hazmat", "no"}}},
        // blanket plus an exclusion but no allowance: category E derives,
        // yet without graded intent the blanket ban is conservatively kept
        {"IJ", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}, {"hazmat", "no"}, {"hazmat:E", "no"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_adr_corridor");
  }
};

gurka::map TruckAdrCorridorTest::map = {};

TEST_F(TruckAdrCorridorTest, PerCodeCorridorGrantsGranularPassage) {
  // the Beneluxtunnel granularity case: codes D and E (and an explicitly
  // unrestricted load) pass the whole category-C corridor
  for (const auto* code : {"D", "E", "(—)"}) {
    SCOPED_TRACE(std::string("code ") + code);
    auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck", hazmat_options(code));
    gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});
  }
}

TEST_F(TruckAdrCorridorTest, PerCodeCorridorStillBlocksItsCategory) {
  // codes B and C - and a hazmat load with no declared code - stay barred
  for (const auto* code : {"B", "C", ""}) {
    SCOPED_TRACE(std::string("code '") + code + "'");
    expect_no_path(map, {"A", "D"}, hazmat_options(code));
  }
}

TEST_F(TruckAdrCorridorTest, PerCodeCorridorIgnoredWithoutHazmat) {
  auto route = gurka::do_action(Options::route, map, {"A", "D"}, "truck");
  gurka::assert::raw::expect_path(route, {"AB", "BC", "CD"});
}

TEST_F(TruckAdrCorridorTest, BlanketPlusAllowanceDerivesCategory) {
  // hazmat=no + hazmat:E=yes is category D: a code-E load passes, a code-D
  // load is blocked, an unrestricted load passes (the blanket is suppressed)
  auto route_e = gurka::do_action(Options::route, map, {"E", "K"}, "truck", hazmat_options("E"));
  gurka::assert::raw::expect_path(route_e, {"EF", "FK"});
  expect_no_path(map, {"E", "K"}, hazmat_options("D"));
  auto route_unrestricted =
      gurka::do_action(Options::route, map, {"E", "K"}, "truck", hazmat_options("(—)"));
  gurka::assert::raw::expect_path(route_unrestricted, {"EF", "FK"});
}

TEST_F(TruckAdrCorridorTest, PlainBlanketStaysBinary) {
  // plain hazmat=no with no per-code sub-keys keeps banning every hazmat
  // load, even an explicitly unrestricted one
  expect_no_path(map, {"G", "L"}, hazmat_options("(—)"));
  auto route = gurka::do_action(Options::route, map, {"G", "L"}, "truck");
  gurka::assert::raw::expect_path(route, {"GH", "HL"});
}

TEST_F(TruckAdrCorridorTest, ExclusionWithoutAllowanceKeepsBlanket) {
  // hazmat=no + hazmat:E=no shows no graded intent (no allowance), so the
  // conservative blanket ban stays even though category E derives
  expect_no_path(map, {"I", "M"}, hazmat_options("(—)"));
  auto route = gurka::do_action(Options::route, map, {"I", "M"}, "truck");
  gurka::assert::raw::expect_path(route, {"IJ", "JM"});
}
