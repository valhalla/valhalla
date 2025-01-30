#include <sif/recost.h>
#include <thor/matrixalgorithm.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

std::string MatrixAlgorithm::FormShape(GraphReader& graphreader,
                                       Api& request,
                                       const std::vector<baldr::GraphId> path_edges,
                                       const PathEdge& source_edge,
                                       const PathEdge& target_edge,
                                       float source_pct,
                                       float target_pct) {

  auto source_vertex = PointLL{source_edge.ll().lng(), source_edge.ll().lat()};
  auto target_vertex = PointLL{target_edge.ll().lng(), target_edge.ll().lat()};
  std::vector<PointLL> points;
  for (uint32_t i = 0; i < path_edges.size(); i++) {
    auto& path_edge = path_edges[i];
    auto is_first_edge = i == 0;
    auto is_last_edge = i == (path_edges.size() - 1);

    graph_tile_ptr tile;
    const auto* de = graphreader.directededge(path_edge, tile);
    auto edge_shp = tile->edgeinfo(de).shape();

    if (is_first_edge || is_last_edge) {
      if (!de->forward())
        std::reverse(edge_shp.begin(), edge_shp.end());

      float total = static_cast<float>(de->length());
      if (is_first_edge && is_last_edge) {
        trim_shape(source_pct * total, source_vertex, target_pct * total, target_vertex, edge_shp);
      } else if (is_first_edge) {
        trim_shape(source_pct * total, source_vertex, total, edge_shp.back(), edge_shp);
      } // last edge
      else {
        trim_shape(0, edge_shp.front(), target_pct * total, target_vertex, edge_shp);
      }

      points.insert(points.end(), edge_shp.begin() + !is_first_edge, edge_shp.end());
    } else {
      if (de->forward()) {
        points.insert(points.end(), edge_shp.begin() + 1, edge_shp.end());
      } else {
        points.insert(points.end(), edge_shp.rbegin() + 1, edge_shp.rend());
      }
    }
  }

  // encode to 6 precision for geojson as well, which the serializer expects
  return encode<decltype(points)>(points, request.options().shape_format() != polyline5 ? 1e6 : 1e5);
}

template <typename LabelType>
void MatrixAlgorithm::FormPath(baldr::GraphReader& graphreader,
                               Api& request,
                               uint32_t source_idx,
                               uint32_t target_idx,
                               const baldr::TimeInfo& time_info,
                               const bool invariant,
                               sif::Cost connection_cost,
                               const uint32_t distance,
                               const std::vector<LabelType>& forward_labels,
                               uint32_t forward_connection,
                               const std::vector<LabelType>& reverse_labels,
                               uint32_t reverse_connection) {

  // bail if (1) the user did not ask for anything that requires forming the path, or (2) if the
  // connection is unfound, or (3) source == target
  if ((!request.options().verbose() &&
       (!has_time_ || (forward_labels.empty() || reverse_labels.empty())) &&
       request.options().shape_format() == no_shape) ||
      connection_cost.secs == 0.f && distance == kMaxCost) {
    return;
  }
  size_t connection_idx = source_idx * request.options().targets().size() + target_idx;
  // set of edges recovered from shortcuts (excluding shortcut's start edges)
  std::unordered_set<GraphId> recovered_inner_edges;

  // A place to keep the path
  std::vector<GraphId> path_edges;

  // Work backwards on the forward path
  graph_tile_ptr tile;
  for (auto edgelabel_index = forward_connection; edgelabel_index != kInvalidLabel;
       edgelabel_index = forward_labels[edgelabel_index].predecessor()) {
    const LabelType& edgelabel = forward_labels[edgelabel_index];

    const DirectedEdge* edge = graphreader.directededge(edgelabel.edgeid(), tile);
    if (edge == nullptr) {
      throw tile_gone_error_t("CostMatrix::RecostPaths failed", edgelabel.edgeid());
    }

    if (edge->is_shortcut()) {
      auto superseded = graphreader.RecoverShortcut(edgelabel.edgeid());
      recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
      std::move(superseded.rbegin(), superseded.rend(), std::back_inserter(path_edges));
    } else
      path_edges.push_back(edgelabel.edgeid());
  }

  // Reverse the list
  std::reverse(path_edges.begin(), path_edges.end());

  // Append the reverse path from the destination - use opposing edges
  // The first edge on the reverse path is the same as the last on the forward
  // path, so get the predecessor in case of a bidirectional expansion
  for (auto edgelabel_index = reverse_labels.empty()
                                  ? reverse_connection
                                  : reverse_labels[reverse_connection].predecessor();
       edgelabel_index != kInvalidLabel;
       edgelabel_index = reverse_labels[edgelabel_index].predecessor()) {
    const LabelType& edgelabel = reverse_labels[edgelabel_index];
    const DirectedEdge* opp_edge = nullptr;
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgelabel.edgeid(), opp_edge, tile);
    if (opp_edge == nullptr) {
      throw tile_gone_error_t("CostMatrix::RecostPaths failed", edgelabel.edgeid());
    }

    if (opp_edge->is_shortcut()) {
      auto superseded = graphreader.RecoverShortcut(opp_edge_id);
      recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
      std::move(superseded.begin(), superseded.end(), std::back_inserter(path_edges));
    } else
      path_edges.emplace_back(std::move(opp_edge_id));
  }

  if (path_edges.empty()) {
    LOG_ERROR("Unable to form path");
    return;
  }

  const auto& source_edge =
      find_correlated_edge(request.options().sources()[source_idx], path_edges.front());
  const auto& target_edge =
      find_correlated_edge(request.options().targets()[target_idx], path_edges.back());
  float source_pct = static_cast<float>(source_edge.percent_along());
  float target_pct = static_cast<float>(target_edge.percent_along());

  // recost if it's a time dependent bidirectional expansion
  if (has_time_ && !forward_labels.empty() && !reverse_labels.empty()) {

    auto edge_itr = path_edges.begin();
    const auto edge_cb = [&edge_itr, &path_edges]() {
      return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
    };

    Cost new_cost{0.f, 0.f};
    const auto label_cb = [&new_cost](const EdgeLabel& label) { new_cost = label.cost(); };

    // recost edges in final path; ignore access restrictions
    try {
      sif::recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct,
                          time_info, invariant, true);
    } catch (const std::exception& e) {
      LOG_ERROR(std::string(name() + " failed to recost final paths: ") + e.what());
      return;
    }

    // update the existing best_connection cost
    connection_cost = new_cost;
  }
  if (request.options().verbose()) {
    request.mutable_matrix()->mutable_begin_lat()->Set(connection_idx, source_edge.ll().lat());
    request.mutable_matrix()->mutable_begin_lon()->Set(connection_idx, source_edge.ll().lng());
    request.mutable_matrix()->mutable_end_lat()->Set(connection_idx, source_edge.ll().lat());
    request.mutable_matrix()->mutable_end_lon()->Set(connection_idx, source_edge.ll().lng());

    // get begin/end heading using the path's begin/end edge shapes
    const DirectedEdge* start_edge =
        graphreader.directededge(static_cast<GraphId>(source_edge.graph_id()), tile);
    std::vector<PointLL> shp = tile->edgeinfo(start_edge).shape();
    if (!start_edge->forward())
      std::reverse(shp.begin(), shp.end());
    request.mutable_matrix()
        ->mutable_begin_heading()
        ->Set(connection_idx, PointLL::HeadingAlongPolyline(shp, start_edge->length() * source_pct));
    const DirectedEdge* end_edge =
        graphreader.directededge(static_cast<GraphId>(target_edge.graph_id()), tile);
    shp = tile->edgeinfo(end_edge).shape();
    if (!end_edge->forward())
      std::reverse(shp.begin(), shp.end());
    request.mutable_matrix()
        ->mutable_end_heading()
        ->Set(connection_idx, PointLL::HeadingAlongPolyline(shp, end_edge->length() * target_pct));
  }

  if (request.options().shape_format() == no_shape)
    return;

  std::string shape =
      FormShape(graphreader, request, path_edges, source_edge, target_edge, source_pct, target_pct);

  *(request.mutable_matrix()->mutable_shapes(connection_idx)) = shape;
}

template void
MatrixAlgorithm::FormPath<sif::EdgeLabel>(baldr::GraphReader& graphreader,
                                          Api& request,
                                          uint32_t source_idx,
                                          uint32_t target_idx,
                                          const baldr::TimeInfo& time_info,
                                          const bool invariant,
                                          sif::Cost connection_cost,
                                          const uint32_t distance,
                                          const std::vector<sif::EdgeLabel>& forward_labels,
                                          uint32_t forward_connection,
                                          const std::vector<sif::EdgeLabel>& reverse_labels,
                                          uint32_t reverse_connection);
template void
MatrixAlgorithm::FormPath<sif::BDEdgeLabel>(baldr::GraphReader& graphreader,
                                            Api& request,
                                            uint32_t source_idx,
                                            uint32_t target_idx,
                                            const baldr::TimeInfo& time_info,
                                            const bool invariant,
                                            sif::Cost connection_cost,
                                            const uint32_t distance,
                                            const std::vector<sif::BDEdgeLabel>& forward_labels,
                                            uint32_t forward_connection,
                                            const std::vector<sif::BDEdgeLabel>& reverse_labels,
                                            uint32_t reverse_connection);
// template void MatrixAlgorithm::FormPath<sif::BDEdgeLabel>();
// template void MyClass::process<double>(double);
} // namespace thor

} // namespace valhalla