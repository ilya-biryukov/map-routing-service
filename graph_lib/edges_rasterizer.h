#pragma once

#include <cmath>
#include <boost/function.hpp>

#include "map_geometry.h"

/**
 * @brief Callback function type for rasterize_edge
 */
typedef boost::function<void (int, int)> put_pixel_f;

/**
 * @brief Rasterizes edge.
 *      Calls put_pixel with coordinates of cells, which contain an edge from p1 to p2
 * @param p1 Point where the edge starts
 * @param p2 Point where the edge ends.
 * @param put_pixel Callback function to be called for each cell
 */
inline void rasterize_edge(grid_point_t const & p1, grid_point_t const & p2, put_pixel_f put_pixel)
{
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();

  double k = dy / dx;
  double b = p1.y() - k * p1.x();

  int i = std::floor(p1.x());
  int j = std::floor(p2.y());

  int max_y = std::floor(std::max(p1.y(), p2.y()));
  int min_y = std::floor(std::min(p1.y(), p2.y()));

  while (i < (int)std::floor(p2.x() + 1.0))
  {
    int next_j = (int)std::floor(k * std::ceil(i + 1.0) + b);

    int min_j = std::min(next_j, j);
    int max_j = std::max(next_j, j);

    if (min_j < min_y)
      min_j = min_y;
    if (max_j > max_y)
      max_j = max_y;

    for (int tmp = min_j; tmp <= max_j; ++tmp)
    {
      put_pixel(i, tmp);
    }

    i += 1;
    j = next_j;
  }
}