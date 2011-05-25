#pragma once

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

#include "graph_data.h"
#include "raw_graph_data.h"

namespace details
{
struct data_accessor_t;

struct edges_iterator: boost::iterator_facade < edges_iterator,// CRTP parameter
      edge_t, // value_type
      boost::bidirectional_traversal_tag, // iterator category
      edge_t > // reference type
{
  // Constructs an invalid iterator(can be used for end())
  edges_iterator() :
      data_accessor_(NULL), source_vertex_id_(INVALID_RAW_VERTEX_ID),
      edge_id_(INVALID_RAW_EDGE_ID)
  {
  }

  edges_iterator(data_accessor_t const * data_accessor,
                 details::raw_vertex_id_t source_id, details::raw_edge_id_t edge_id) :
      data_accessor_(data_accessor), source_vertex_id_(source_id),
      edge_id_(edge_id)
  {
  }

  edges_iterator(edges_iterator const & other) :
      data_accessor_(other.data_accessor_),
      source_vertex_id_(other.source_vertex_id_), edge_id_(other.edge_id_)
  {
  }
  
  size_t get_edge_id() const
  {
    return edge_id_;
  }
private:
  void increment();
  void decrement();
  edge_t dereference() const;
  bool equal(edges_iterator const & other) const
  {
    if (edge_id_ == INVALID_RAW_EDGE_ID && other.edge_id_ == INVALID_RAW_EDGE_ID)
      return true;

    return data_accessor_ == other.data_accessor_ && edge_id_ == other.edge_id_
           && source_vertex_id_ == other.source_vertex_id_;
  }

data_accessor_t const * data_accessor_;
  details::raw_vertex_id_t source_vertex_id_;
  details::raw_edge_id_t edge_id_;

  friend class boost::iterator_core_access;
};

struct data_accessor_t
{
  typedef boost::iterator_range<details::edges_iterator> edges_range;

  data_accessor_t(void * raw_graph_data);
  
  vertex_t get_vertex_by_id(vertex_id_t id) const;
  size_t get_vertices_count() const;
  size_t get_edges_count() const;
  edges_range get_incident_edges(vertex_id_t vertex_id, 
                                 edge_type_t edges_type) const;
private:
  details::raw_edge_t * raw_edge_by_id(details::raw_edge_id_t id) const;
  details::raw_vertex_t * raw_vertex_by_id(details::raw_vertex_id_t id) const;

  details::raw_vertices_header_t * vertices_header_;
  details::raw_edges_header_t * edges_header_;

  friend class details::edges_iterator;
};
} // namespace details


// Implementation
namespace details
{
inline void edges_iterator::increment()
{
  if (edge_id_ != INVALID_RAW_EDGE_ID)
  {
    ++edge_id_;
  }
}

inline void edges_iterator::decrement()
{
  if (edge_id_ != INVALID_RAW_EDGE_ID)
  {
    --edge_id_;
  }
}

inline edge_t edges_iterator::dereference() const
{
  details::raw_edge_t * edge = data_accessor_->raw_edge_by_id(edge_id_);
  return edge_t(source_vertex_id_, edge->destination, edge->weight);
}

inline data_accessor_t::data_accessor_t(void * raw_graph_data)
  : vertices_header_(static_cast<details::raw_vertices_header_t *>(raw_graph_data))
{
  edges_header_ = reinterpret_cast<raw_edges_header_t *>(
    &vertices_header_->vertices[vertices_header_->vertices_count]);
}
} // namespace details
