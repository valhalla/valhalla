#ifndef VALHALLA_BALDR_MERGE_H_
#define VALHALLA_BALDR_MERGE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace baldr {
namespace merge {

// a segment is a collection of a start node, end node and the edge between
// them. this is used when creating paths.
struct segment {
  GraphId m_start, m_edge, m_end;
  segment(GraphId start, GraphId edge, GraphId end);
  GraphId start() const { return m_start; }
  GraphId end() const { return m_end; }
  GraphId edge() const { return m_edge; }
};

// a path is a start node, end node and a collection of contiguous edges between
// them. it is created by concatenating segments.
struct path {
  GraphId m_start, m_end;
  std::deque<GraphId> m_edges;

  explicit path(segment s);
  explicit path(GraphId node_id);

  void push_back(segment s);
  void push_front(segment s);
};

/**
 * Read the graph and merge all compatible edges, calling the provided function
 * with each path that has been found. Each edge in the graph will be part of
 * exactly one path, but paths may contain multiple edges.
 *
 * @param reader The graph to traverse.
 * @param func The function to execute for each discovered path.
 */
void merge(GraphReader &reader, std::function<void(const path &)> func);

}
}
}

#endif  // VALHALLA_BALDR_MERGE_H_
