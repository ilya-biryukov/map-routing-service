#pragma once

#include <boost/cstdint.hpp>

namespace details
{  
typedef boost::uint64_t raw_vertex_id_t;
typedef boost::uint64_t raw_edge_id_t;

raw_vertex_id_t const INVALID_RAW_VERTEX_ID = -1L;
raw_edge_id_t const INVALID_RAW_EDGE_ID = -1L;

struct raw_vertex_t
{
  double lon;
  double lat;
  raw_edge_id_t edges_begin_id;
  raw_edge_id_t edges_end_id;
  raw_edge_id_t transposed_edges_begin_id; // transposed_edges_start = edges_end
  raw_edge_id_t transposed_edges_end_id;
};


struct raw_edge_t
{
  raw_vertex_id_t destination;
  double weight;
};


struct raw_vertices_header_t
{
  boost::uint64_t vertices_count;
  raw_vertex_t vertices[];
};


struct raw_edges_header_t
{
  boost::uint64_t edges_count;
  raw_edge_t edges[];
};
} // namespace details