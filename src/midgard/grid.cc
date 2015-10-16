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

      cell_width_recip = divisions / super_cell.Width();
      cell_height_recip = divisions / super_cell.Height();

      //make the cells
      cells.reserve(divisions * divisions);
      for(size_t i = 0; i < divisions; ++i) {
        for(size_t j = 0; j < divisions; ++j) {
          coord_t cell_min(min.first + (static_cast<x_t>(i) / divisions) * super_cell.Width(),
                           min.second + (static_cast<y_t>(j) / divisions) * super_cell.Height());
          coord_t cell_max(min.first + (static_cast<x_t>(i + 1) / divisions) * super_cell.Width(),
                           min.second + (static_cast<y_t>(j + 1) / divisions) * super_cell.Height());
          cells.emplace_back(cell_min, cell_max);
        }
      }
    }

    template <class coord_t>
    template <class container_t>
    std::unordered_set<size_t> grid<coord_t>::intersect(const container_t& linestring) const {
      std::unordered_set<size_t> indices;
      indices.reserve(cells.size());

      //for each segment
      for(auto v = std::next(linestring.cbegin()); v != linestring.cend(); std::advance(v, 1)) {
        //is it in the super cell
        auto u = std::prev(v);
        if(!super_cell.Intersects(*u, *v))
          continue;

        //get the subset of cells to iterate over
        size_t x_start = static_cast<size_t>((clamp(u->first, super_cell.minx(), super_cell.maxx()) - super_cell.minx()) * cell_width_recip);
        size_t x_end = static_cast<size_t>((clamp(v->first, super_cell.minx(), super_cell.maxx()) - super_cell.minx()) * cell_width_recip);
        if(x_start > x_end)
          std::swap(x_start, x_end);
        size_t y_start = static_cast<size_t>((clamp(u->second, super_cell.miny(), super_cell.maxy()) - super_cell.miny()) * cell_height_recip);
        size_t y_end = static_cast<size_t>((clamp(v->second, super_cell.miny(), super_cell.maxy()) - super_cell.miny()) * cell_height_recip);
        if(y_start > y_end)
          std::swap(y_start, y_end);

        //for each cell in the subset
        for(; x_start <= x_end; ++x_start) {
          for(; y_start <= y_end; ++y_start) {
            size_t index = y_start * divisions + x_start;
            if(cells[index].Intersects(*u, *v)) {
              indices.insert(index);
            }
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

    //explicit instantiation
    template class grid<Point2>;
    template class grid<PointLL>;
    template class std::unordered_set<size_t> grid<Point2>::intersect(const std::list<Point2>&) const;
    template class std::unordered_set<size_t> grid<PointLL>::intersect(const std::list<PointLL>&) const;
    template class std::unordered_set<size_t> grid<Point2>::intersect(const std::vector<Point2>&) const;
    template class std::unordered_set<size_t> grid<PointLL>::intersect(const std::vector<PointLL>&) const;

  }
}
