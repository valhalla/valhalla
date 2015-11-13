#include "midgard/grid.h"
#include "midgard/util.h"

#include <list>

using namespace valhalla::midgard;

namespace {


}

namespace valhalla {
  namespace midgard {

    template <class coord_t>
    grid<coord_t>::grid(const AABB2<coord_t>& extents, size_t divisions):
      grid(extents.minpt(), extents.maxpt(), divisions) {
    }

    template <class coord_t>
    grid<coord_t>::grid(const coord_t& min, const coord_t& max, size_t divisions):
      divisions(divisions), super_cell(min, max) {

      x_index_coef = divisions / super_cell.Width();
      y_index_coef = divisions / super_cell.Height();

      //make the cells
      cells.reserve(divisions * divisions);
      for(size_t y = 0; y < divisions; ++y) {
        for(size_t x = 0; x < divisions; ++x) {
          coord_t cell_min(min.first + (static_cast<x_t>(x) / divisions) * super_cell.Width(),
                           min.second + (static_cast<y_t>(y) / divisions) * super_cell.Height());
          coord_t cell_max(min.first + (static_cast<x_t>(x + 1) / divisions) * super_cell.Width(),
                           min.second + (static_cast<y_t>(y + 1) / divisions) * super_cell.Height());
          cells.emplace_back(cell_min, cell_max);
        }
      }
    }

    template <class coord_t>
    template <class container_t>
    std::unordered_set<size_t> grid<coord_t>::intersect(const container_t& linestring, bool& uncontained) const {
      std::unordered_set<size_t> indices;
      indices.reserve(cells.size());
      uncontained = false;

      //for each segment
      for(auto u = linestring.cbegin(); u != linestring.cend(); std::advance(u, 1)) {
        //figure out what the segment is
        auto v = std::next(u);
        if(v == linestring.cend())
          v = u;

        //this is outside of the cell
        if(!super_cell.Contains(*u) || !super_cell.Contains(*v)) {
          uncontained = true;
          //TODO: get neighbors
          //this is completely outside of the cell
          if(!super_cell.Intersects(*u, *v))
            continue;
        }

        //figure out the subset of the grid that this segment overlaps
        size_t x_start = static_cast<size_t>((clamp(u->first, super_cell.minx(), super_cell.maxx()) - super_cell.minx()) * x_index_coef);
        size_t y_start = static_cast<size_t>((clamp(u->second, super_cell.miny(), super_cell.maxy()) - super_cell.miny()) * y_index_coef);
        size_t x_end = static_cast<size_t>((clamp(v->first, super_cell.minx(), super_cell.maxx()) - super_cell.minx()) * x_index_coef);
        size_t y_end = static_cast<size_t>((clamp(v->second, super_cell.miny(), super_cell.maxy()) - super_cell.miny()) * y_index_coef);
        if(x_start > x_end)
          std::swap(x_start, x_end);
        if(y_start > y_end)
          std::swap(y_start, y_end);
        x_end = std::min(x_end + 1, divisions);
        y_end = std::min(y_end + 1, divisions);

        //loop over the subset of the grid that intersects the bbox formed by the linestring
        for(; y_start < y_end; ++y_start) {
          for(size_t x = x_start; x < x_end; ++x) {
            size_t index = y_start * divisions + x;
            if(cells[index].Intersects(*u, *v))
              indices.insert(index);
          }
        }
      }

      //give them back
      return indices;
    }

    template <class coord_t>
    std::unordered_set<size_t> grid<coord_t>::intersect(const coord_t& center, const float radius) const {
      std::unordered_set<size_t> indices;
      indices.reserve(cells.size());

      //super cell doesnt intersect it then nothing inside it can
      if(!super_cell.Intersects(center, radius))
        return indices;

      //for each cell
      //TODO: flood fill can terminate early and has equivalent worst case would
      //be helpful to seed it with intersection point from super_cell.Intersect()
      for(size_t i = 0; i < cells.size(); ++i) {
        //does it intersect
        if(cells[i].Intersects(center, radius)) {
          indices.insert(i);
        }
      }

      //give them back
      return indices;
    }

    template <class coord_t>
    size_t grid<coord_t>::size() const {
      return cells.size();
    }

    template <class coord_t>
    const AABB2<coord_t>& grid<coord_t>::extent() const {
      return super_cell;
    }

    //explicit instantiation
    template class grid<Point2>;
    template class grid<PointLL>;

    template class std::unordered_set<size_t> grid<Point2>::intersect(const std::list<Point2>&, bool&) const;
    template class std::unordered_set<size_t> grid<PointLL>::intersect(const std::list<PointLL>&, bool&) const;
    template class std::unordered_set<size_t> grid<Point2>::intersect(const std::vector<Point2>&, bool&) const;
    template class std::unordered_set<size_t> grid<PointLL>::intersect(const std::vector<PointLL>&, bool&) const;

  }
}
