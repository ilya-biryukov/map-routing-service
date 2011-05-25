#include "spatial_index.h"

#include <cmath>
#include <iostream>

#include <boost/bind.hpp>

#include "graph_data.h"
#include "edges_rasterizer.h"
#include "map_geometry.h"

spatial_index_t::spatial_index_t (graph_t const & graph, size_t lon_count, size_t lat_count)
    : lon_count_(lon_count), lat_count_(lat_count), buckets_map_(graph.get_vertices_count()),
      graph_(graph)
{
  build_index(graph);  
}

bucket_id_t spatial_index_t::get_bucket_id(vertex_data_t pos) const
{
  double lon_diff = 180. / lon_count_;
  double lat_diff = 360. / lat_count_;

  return static_cast<bucket_id_t>((pos.lon() + 180.) / lon_diff)
         + static_cast<bucket_id_t>((pos.lat() + 90.) / lat_diff) * lon_count_;
}


bucket_id_t spatial_index_t::get_bucket_id(int x, int y) const
{
  return y * lon_count_ + x;
}

grid_point_t spatial_index_t::get_grid_point(wgs82_point_t const & geodata) const
{
  double dx = 360.0 / lon_count_;
  double dy = 180.0 / lat_count_;
  return grid_point_t((180.0 + geodata.lon()) / 360.0 * lon_count_,
                      (90.0 + geodata.lat()) / 360.0 * lat_count_);    
}

void spatial_index_t::build_index(graph_t const & graph)
{
  for (size_t i = 0; i != graph.get_vertices_count(); ++i)
  {
    vertex_t v1(graph.get_vertex_by_id(i));

    graph_t::edges_range edges = graph.get_incident_edges(v1, EDGE_NORMAL);
    for (graph_t::edges_range::const_iterator eit = edges.begin();
         eit != edges.end();
         ++eit)
    {
      vertex_t v2(graph.get_vertex_by_id(eit->destination()));
      
      grid_point_t s(get_grid_point(v1.data()));
      grid_point_t t(get_grid_point(v2.data()));
      
      rasterize_edge(s, t, boost::bind(&spatial_index_t::add_to_index, this, _1, _2, *eit));
    }
  }
}


struct extract_edge_f
{
  typedef edge_t result_type;
  
  edge_t operator () (std::pair<bucket_id_t, edge_t> const & pair)
  {
      return pair.second;
  }
};

void spatial_index_t::for_all_nearest_edges(vertex_data_t const & pos, boost::function<void (edge_t)> fn) const
{
  grid_point_t point(get_grid_point(pos));
  
  std::pair<buckets_map_t::const_iterator, buckets_map_t::const_iterator> range = 
    buckets_map_.equal_range(get_bucket_id(point.x(), point.y()));
  std::for_each(range.first, range.second, boost::bind(fn, boost::bind(extract_edge_f(), _1)));  
}

void spatial_index_t::add_to_index(int x, int y, const edge_t& edge)
{
  buckets_map_.insert(std::make_pair(get_bucket_id(x, y), edge));
}

