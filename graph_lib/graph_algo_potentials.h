#pragma once

#include "graph_data.h"
#include "map_geometry.h"

namespace details
{
struct potential_dist_f
{
  double operator () (vertex_t v1, vertex_t v2)
  {
    return get_distance(v1.data(), v2.data());
  }
};
} // namespace details