#include "data_accessor.h"

namespace details
{
vertex_t data_accessor_t::get_vertex_by_id(vertex_id_t id) const
{
  details::raw_vertex_t * raw_v = raw_vertex_by_id(id);
  if (raw_v == NULL)
    return vertex_t();
  else
    return vertex_t(id, vertex_data_t(raw_v->lon, raw_v->lat));
}

size_t data_accessor_t::get_vertices_count() const
{
  return vertices_header_->vertices_count;
}

size_t data_accessor_t::get_edges_count() const
{
  return edges_header_->edges_count;
}

data_accessor_t::edges_range data_accessor_t::get_incident_edges(vertex_id_t vertex_id, 
                                                                 edge_type_t edges_type) const
{
  details::raw_vertex_t * raw_v = raw_vertex_by_id(vertex_id);

  if (raw_v == NULL)
  {
    return edges_range(details::edges_iterator(), details::edges_iterator());
  }
  
  raw_edge_id_t eid_begin;
  raw_edge_id_t eid_end;
  if (edges_type == EDGE_REVERSED)
  {
    eid_begin = raw_v->transposed_edges_begin_id;
    eid_end = raw_v->transposed_edges_end_id;    
  }
  else // edges_type == EDGE_NORMAL or wrong edges_type
  {
    eid_begin = raw_v->edges_begin_id;
    eid_end = raw_v->edges_end_id;
  }
  
  return edges_range(details::edges_iterator(this, vertex_id, eid_begin),
                     details::edges_iterator(this, vertex_id, eid_end));
}

details::raw_edge_t * data_accessor_t::raw_edge_by_id(details::raw_edge_id_t id) const
{
  if (id >= edges_header_->edges_count)
    return NULL;

  return &edges_header_->edges[id];
}

details::raw_vertex_t * data_accessor_t::raw_vertex_by_id(details::raw_vertex_id_t id) const
{
  if (id >= vertices_header_->vertices_count)
    return NULL;

  return &vertices_header_->vertices[id];
}
} // namespace details