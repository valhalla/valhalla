#include "loki/linear_cost_factors.h"
#include "exceptions.h"
#include "thor/route_matcher.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

// first and last edges of a cost factor line covering less than this are dropped
constexpr double kMinCostFactorEdgeLength = 1.0; // meters

/**
 * Adds a shortcut to the cost factor edges given one
 * of its constituents
 */
void add_shortcut(GraphReader& reader,
                  GraphId shortcut,
                  valhalla::Costing_Options* options,
                  valhalla::CostFactorEdge* cost_factor) {

  // for ignoring access restrictions, we don't care if it's
  // a partial, it applies to the whole edge
  if (cost_factor->ignore_access_restrictions()) {
    auto* exclude_edge = options->add_exclude_edges();
    exclude_edge->set_id(shortcut.value);
    return;
  }
  GraphId edge = static_cast<GraphId>(cost_factor->id());
  graph_tile_ptr tile = reader.GetGraphTile(shortcut);
  // it's part of a shortcut
  auto constituents = reader.RecoverShortcut(shortcut);
  auto* shortcut_edge = tile->directededge(shortcut);

  tile = reader.GetGraphTile(edge);
  auto* current_edge = tile->directededge(edge);

  // walk the base edges until we find ours
  uint64_t accumulated_length = 0;
  for (const auto& constituent : constituents) {
    if (edge == constituent)
      break;

    tile = reader.GetGraphTile(constituent, tile);
    if (!tile)
      break;

    auto* de = tile->directededge(constituent);
    accumulated_length += de->length();
  }
  auto* e = options->add_cost_factor_edges();
  e->set_id(shortcut);
  e->set_factor(cost_factor->factor());
  e->set_start(static_cast<double>(accumulated_length + (static_cast<double>(current_edge->length()) *
                                                         cost_factor->start())) /
               static_cast<double>(shortcut_edge->length()));
  e->set_end(static_cast<double>(accumulated_length +
                                 (static_cast<double>(current_edge->length()) * cost_factor->end())) /
             static_cast<double>(shortcut_edge->length()));
}

} // namespace

namespace valhalla {
namespace loki {

void add_cost_factor_edges(const sif::mode_costing_t& mode_costing,
                           const sif::TravelMode& mode,
                           GraphReader& reader,
                           valhalla::Options& options,
                           double min_allowed_factor,
                           uint64_t max_allowed_edges) {
  Costing_Options* costing_options =
      options.mutable_costings()->find(options.costing_type())->second.mutable_options();

  // keep track of how many edges we're adding
  uint64_t edge_count = 0;

  for (auto& line : *options.mutable_cost_factor_lines()) {
    std::vector<std::vector<PathInfo>> legs;
    if (!RouteMatcher::FormPath(mode_costing, mode, reader, line, false, /* use_shortcuts=*/true,
                                legs)) {
      throw valhalla_exception_t{233};
    }
    for (const auto& leg : legs) {
      for (size_t i = 0; i < leg.size(); ++i) {
        if (edge_count > max_allowed_edges)
          throw valhalla_exception_t{234};
        auto& path_info = leg[i];
        bool is_first = i == 0;
        bool is_last = i == leg.size() - 1;
        if (is_first && is_last) { // trivial path
          edge_count++;
          auto* e = costing_options->add_cost_factor_edges();
          e->set_id(path_info.edgeid);
          e->set_factor(line.cost_factor());
          e->set_ignore_access_restrictions(line.ignore_access_restrictions());
          for (const auto& edge : line.locations(0).correlation().edges()) {
            if (path_info.edgeid == edge.graph_id()) {
              e->set_start(edge.percent_along());
              break;
            }
          }
          for (const auto& edge : line.locations(1).correlation().edges()) {
            if (path_info.edgeid == edge.graph_id()) {
              e->set_end(edge.percent_along());
              break;
            }
          }
          auto shortcut = reader.GetShortcut(path_info.edgeid);
          if (shortcut.is_valid()) {
            add_shortcut(reader, shortcut, costing_options, e);
          }
        } else if (is_first || is_last) { // beginning or end edge
          for (const auto& edge :
               line.locations(static_cast<size_t>(is_last)).correlation().edges()) {
            if (path_info.edgeid == edge.graph_id()) {
              double start = is_first ? edge.percent_along() : 0.;
              double end = is_last ? edge.percent_along() : 1.;
              // an endpoint just off a node also correlates to the adjacent edges, skip those small
              // parts
              const auto* de = reader.directededge(path_info.edgeid);
              if (de && de->length() * (end - start) < kMinCostFactorEdgeLength) {
                break;
              }
              edge_count++;
              auto* e = costing_options->add_cost_factor_edges();
              e->set_id(path_info.edgeid);
              // apply the minimum allowed value specified in the config
              e->set_factor(std::max(line.cost_factor(), min_allowed_factor));
              e->set_ignore_access_restrictions(line.ignore_access_restrictions());
              e->set_start(start);
              e->set_end(end);
              auto shortcut = reader.GetShortcut(path_info.edgeid);
              if (shortcut.is_valid()) {
                add_shortcut(reader, shortcut, costing_options, e);
              }
              break;
            }
          }
        } else { // intermediate edges
          edge_count++;
          auto* e = costing_options->add_cost_factor_edges();
          e->set_id(path_info.edgeid);
          e->set_factor(std::max(line.cost_factor(), min_allowed_factor));
          e->set_ignore_access_restrictions(line.ignore_access_restrictions());
          e->set_start(0.);
          e->set_end(1.);

          // if it's a shortcut, also add all of its constituent edges
          if (path_info.is_shortcut) {
            auto constituents = reader.RecoverShortcut(path_info.edgeid);
            for (const auto& constituent : constituents) {
              edge_count++;
              auto* e = costing_options->add_cost_factor_edges();
              e->set_id(constituent);
              e->set_factor(std::max(line.cost_factor(), min_allowed_factor));
              e->set_ignore_access_restrictions(line.ignore_access_restrictions());
              e->set_start(0);
              e->set_end(1);
            }
          } else {
            // if it's not a shortcut, it may be part of one
            // TODO: this is an expensive operation, since we need to expand the graph
            // a little, can't we persist this information somehow?
            auto shortcut = reader.GetShortcut(path_info.edgeid);
            if (shortcut.is_valid()) {
              add_shortcut(reader, shortcut, costing_options, e);
            }
          }
        }
      }
    }
  }
}

} // namespace loki
} // namespace valhalla
