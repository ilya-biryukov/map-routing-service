#pragma once

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/noncopyable.hpp>

#include "graph_data.h"
#include "data_accessor.h"

/**
 * @brief Graph with additional geographic information on vertices
 */
struct graph_t : boost::noncopyable
{
  typedef details::data_accessor_t::edges_range edges_range;

  graph_t(std::string const & path_to_graph);
  
  edges_range get_incident_edges(vertex_t const & v, edge_type_t edges_type) const;
  size_t get_vertices_count() const;
  size_t get_edges_count() const;
  vertex_t get_vertex_by_id(vertex_id_t id) const;

private:
  void init(std::string const & path_to_graph, size_t graph_data_size);
  
  boost::shared_ptr<details::data_accessor_t> data_accessor_ptr_;
  boost::shared_ptr<boost::interprocess::mapped_region> mapped_data_ptr_;
};


// Implementation
inline graph_t::edges_range graph_t::get_incident_edges(vertex_t const & v, 
                                                        edge_type_t edges_type) const
{
  return data_accessor_ptr_->get_incident_edges(v.id(), edges_type);
}

inline size_t graph_t::get_vertices_count() const
{
  return data_accessor_ptr_->get_vertices_count();
}

inline size_t graph_t::get_edges_count() const
{
  return data_accessor_ptr_->get_edges_count();
}

inline vertex_t graph_t::get_vertex_by_id(vertex_id_t id) const
{
  return data_accessor_ptr_->get_vertex_by_id(id);
}