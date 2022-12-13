#pragma once
/******************************************************************************
 * End-to-end tests
 *
 * These include:
 *   1. Parsing an OSM map
 *   2. Generating tiles
 *   3. Calculating routes on tiles
 *   4. Verify the expected route
 ******************************************************************************/
#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "loki/worker.h"
#include "midgard/constants.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "mjolnir/util.h"
#include "odin/worker.h"
#include "proto/trip.pb.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "tyr/serializers.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/output_iterator.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/object_pointer_collection.hpp>
#include <osmium/osm/object_comparisons.hpp>

#include <regex>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

namespace valhalla {
namespace gurka {

using nodelayout = std::map<std::string, midgard::PointLL>;

struct map {
  boost::property_tree::ptree config;
  nodelayout nodes;
};

using ways = std::map<std::string, std::map<std::string, std::string>>;
using nodes = std::map<std::string, std::map<std::string, std::string>>;

enum relation_member_type { node_member, way_member };
struct relation_member {
  relation_member_type type;
  std::string ref;
  std::string role;
};
struct relation {
  std::vector<relation_member> members;
  std::map<std::string, std::string> tags;
};

using relations = std::vector<relation>;

namespace detail {

/**
 * Given a string that's an "ASCII map", will decide on coordinates
 * for the nodes drawn on the grid.
 *
 * @returns a dictionary of node IDs to lon/lat values
 */
nodelayout map_to_coordinates(const std::string& map,
                              const double gridsize_metres,
                              const midgard::PointLL& topleft = {0, 0});

/**
 * Given a map of node locations, ways, node properties and relations, will
 * generate an OSM compatible PBF file, suitable for loading into Valhalla
 */
void build_pbf(const nodelayout& node_locations,
               const ways& ways,
               const nodes& nodes,
               const relations& relations,
               const std::string& filename,
               const uint64_t initial_osm_id = 0,
               const bool strict = true);

/**
 * Extract list of edge names from route result.
 * @param result the result of a /route or /match request
 * @return list of edge names
 */
std::vector<std::vector<std::string>> get_paths(const valhalla::Api& result);
} // namespace detail

/**
 * Given a node layout, set of ways, node properties and relations, generates an OSM PBF file,
 * and builds a set of Valhalla tiles for it.
 *
 * @param layout the locations of all the nodes
 * @param ways the way definitions (which nodes are connected, and their properties
 * @param nodes properties on any of the defined nodes
 * @param relations OSM relations that related nodes and ways together
 * @param workdir where to build the PBF and the tiles
 * @param config_options optional key value pairs where the key is ptree style dom traversal and
 *        the value is the value to put into the config. You can do things like add timezones database
 *        path
 * @return a map object that contains the Valhalla config (to pass to GraphReader) and node layout
 *         (for converting node names to coordinates)
 */
map buildtiles(const nodelayout& layout,
               const ways& ways,
               const nodes& nodes,
               const relations& relations,
               const std::string& workdir,
               const std::unordered_map<std::string, std::string>& config_options = {
                   {"mjolnir.concurrency", "1"}});

/**
 * Given a node layout, set of ways, node properties and relations, generates an OSM PBF file,
 * and builds a set of Valhalla tiles for it.
 *
 * @param layout the locations of all the nodes
 * @param ways the way definitions (which nodes are connected, and their properties
 * @param nodes properties on any of the defined nodes
 * @param relations OSM relations that related nodes and ways together
 * @param config fully fledged valhalla config, the mjolnir section is used to build tiles
 * @return a map object that contains the Valhalla config (to pass to GraphReader) and node layout
 *         (for converting node names to coordinates)
 */
map buildtiles(const nodelayout& layout,
               const ways& ways,
               const nodes& nodes,
               const relations& relations,
               const boost::property_tree::ptree& config);

/**
 * Finds a directed edge in the generated map.  Helpful because the IDs assigned
 * to edges depends on the shape of the map.
 *
 * @param reader a reader configured to read graph tiles
 * @param nodes a lookup table from node names to coordinates
 * @param way_name the way name you want a directed edge for
 * @param end_node the node that should be the target of the directed edge you want
 * @param tile_id optional tile_id to limit the search to
 * @return the directed edge that matches, or nullptr if there was no match
 */
std::tuple<const baldr::GraphId,
           const baldr::DirectedEdge*,
           const baldr::GraphId,
           const baldr::DirectedEdge*>
findEdge(valhalla::baldr::GraphReader& reader,
         const nodelayout& nodes,
         const std::string& way_name,
         const std::string& end_node,
         const baldr::GraphId& tile_id = baldr::GraphId{});

/**
 * Finds an edge in the graph based on its begin and end node names
 *
 * @param reader           graph reader to look up tiles and edges
 * @param begin_node_name  name of the begin node
 * @param end_node_name    name of the end node
 * @return the edge_id and its edge
 */
std::tuple<const baldr::GraphId, const baldr::DirectedEdge*>
findEdgeByNodes(valhalla::baldr::GraphReader& reader,
                const nodelayout& nodes,
                const std::string& begin_node_name,
                const std::string& end_node_name);

/**
 * Finds a node in the graph based on its node name
 *
 * @param reader           graph reader to look up tiles and edges
 * @param nodes            a lookup table from node names to coordinates
 * @param node_name        name of the node
 * @return the node_id
 */
baldr::GraphId
findNode(valhalla::baldr::GraphReader& reader, const nodelayout& nodes, const std::string& node_name);

std::string do_action(const map& map,
                      valhalla::Api& api,
                      std::shared_ptr<valhalla::baldr::GraphReader> reader = {});

valhalla::Api do_action(const valhalla::Options::Action& action,
                        const map& map,
                        const std::string& request_json,
                        std::shared_ptr<valhalla::baldr::GraphReader> reader = {},
                        std::string* json = nullptr);

valhalla::Api do_action(const valhalla::Options::Action& action,
                        const map& map,
                        const std::vector<std::string>& waypoints,
                        const std::string& costing,
                        const std::unordered_map<std::string, std::string>& options = {},
                        std::shared_ptr<valhalla::baldr::GraphReader> reader = {},
                        std::string* json = nullptr,
                        const std::string& stop_type = "break",
                        std::string* request_json = nullptr);

// overload for /sources_to_targets
valhalla::Api do_action(const valhalla::Options::Action& action,
                        const map& map,
                        const std::vector<std::string>& sources,
                        const std::vector<std::string>& targets,
                        const std::string& costing,
                        const std::unordered_map<std::string, std::string>& options = {},
                        std::shared_ptr<valhalla::baldr::GraphReader> reader = {},
                        std::string* response = nullptr,
                        std::string* request_json = nullptr);

/* Returns the raw_result formatted as a JSON document in the given format.
 *
 * @param raw_result the result of a /route or /match request
 * @param format the response format to use for the JSON document
 * @return A JSON document created from serialized raw_result. Caller should
 * call HasParseError() on the returned document to verify its validity.
 */
rapidjson::Document convert_to_json(valhalla::Api& raw_result, valhalla::Options_Format format);

// TODO: bidirectional edges overlap shapes so its hard to visualize them
/**
 * Dumps the gurka map to geojson including edges and nodes. Properties will be street names and node
 * names. Edge ids are also included
 * @param graph    the gurka map
 * @return geojson string
 */
std::string dump_geojson_graph(const map& graph);

namespace assert {
namespace osrm {

/**
 * Tests if a found path traverses the expected steps in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the step roads the path should traverse in order
 * @param dedupe whether subsequent same-name roads should appear multiple times or not (default not)
 */
void expect_steps(valhalla::Api& raw_result,
                  const std::vector<std::string>& expected_names,
                  bool dedupe = true,
                  const std::string& route_name = "routes");

/**
 * Tests if the result, which may be comprised of multiple routes,
 * have summaries that match the expected_summaries.
 *
 * Note: For simplicity's sake, this logic looks at the first leg of each route.
 *
 * @param result the result of a /route or /match request
 * @param expected_summaries the route/leg summaries expected
 */
void expect_summaries(valhalla::Api& raw_result, const std::vector<std::string>& expected_summaries);

/**
 * Tests if a found path traverses the expected roads in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the roads the path should traverse in order
 * @param dedupe whether subsequent same-name roads should appear multiple times or not (default not)
 */
void expect_match(valhalla::Api& raw_result,
                  const std::vector<std::string>& expected_names,
                  bool dedupe = true);
} // namespace osrm

namespace raw {
/**
 * Tests whether the expected sequence of maneuvers is emitted for the route.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param expected_maneuvers all the maneuvers expected in the DirectionsLeg for the route
 */
void expect_maneuvers(const valhalla::Api& result,
                      const std::vector<valhalla::DirectionsLeg_Maneuver_Type>& expected_maneuvers);

/**
 * Tests whether the expected sequence of maneuver begin path indexes is emitted for the route.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param expected_indexes all the maneuver begin path indexes expected in the DirectionsLeg
 *                         for the route
 */
void expect_maneuver_begin_path_indexes(const valhalla::Api& result,
                                        const std::vector<uint32_t>& expected_indexes);

/**
 * Tests whether the expected set of instructions is emitted for the specified maneuver index.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param maneuver_index the specified maneuver index to inspect
 * @param expected_text_instruction the expected text instruction
 * @param expected_verbal_succinct_transition_instruction the expected verbal succinct transition
 *                                                     instruction
 * @param expected_verbal_transition_alert_instruction the expected verbal transition alert
 *                                                     instruction
 * @param expected_verbal_pre_transition_instruction the expected verbal pre-transition instruction
 * @param expected_verbal_post_transition_instruction the expected verbal post-transition instruction
 */
void expect_instructions_at_maneuver_index(
    const valhalla::Api& result,
    int maneuver_index,
    const std::string& expected_text_instruction,
    const std::string& expected_verbal_succinct_transition_instruction,
    const std::string& expected_verbal_transition_alert_instruction,
    const std::string& expected_verbal_pre_transition_instruction,
    const std::string& expected_verbal_post_transition_instruction);

void expect_path_length(const valhalla::Api& result,
                        const float expected_length_km,
                        const float error_margin = 0);

void expect_eta(const valhalla::Api& result,
                const float expected_eta_seconds,
                const float error_margin = 0);

/**
 * Tests if a found path traverses the expected edges in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the edges the path should traverse in order
 * @param message the message prints if a test is failed
 */
void expect_path(const valhalla::Api& result,
                 const std::vector<std::string>& expected_names,
                 const std::string& message = "");

} // namespace raw
} // namespace assert

} // namespace gurka
} // namespace valhalla
