#pragma once

#include <fstream>
#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/unordered_map.hpp>
#include <boost/function.hpp>

#include "graph.h"
#include "graph_data.h"
#include "edges_rasterizer.h"

/**
 * @brief Type for bucket_id
 */
typedef uint32_t bucket_id_t;

/**
 * @brief A spatial index used for fast edge lookup
 */
struct spatial_index_t
{
  /**
   * @brief Builds new spatial_index for graph with uniform grid.
   * @param graph Graph for indexing
   * @param lon_count Number of cells in a grid for lontitude
   * @param lat_count Number of cells in a grid for latitude
   */
  spatial_index_t(graph_t const & graph, size_t lon_count, size_t lat_count);  
  
  /**
   * @brief Applies fn to each edge in an index that is near pos.
   *    Generally speaking, applies fn to all points which all not further than 1 grid cell away 
   *    from pos.
   * @param pos Position for edges lookup
   * @param fn Callback function which will be called for found edges
   */
  void for_all_nearest_edges(vertex_data_t const & pos, boost::function<void (edge_t)> fn) const;
private:
  typedef boost::unordered_multimap<bucket_id_t, edge_t> buckets_map_t;
  
  bucket_id_t get_bucket_id(vertex_data_t pos) const;
  bucket_id_t get_bucket_id(int x, int y) const;
  
  grid_point_t get_grid_point(wgs82_point_t const & geodata) const;
  
  void build_index(graph_t const & graph);
  void add_to_index(int x, int y, edge_t const & edge);
  
  size_t          lon_count_;
  size_t          lat_count_;
  buckets_map_t   buckets_map_;
  graph_t const & graph_;
};