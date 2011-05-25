#pragma once

#include <boost/bind.hpp>

#include "graph.h"
#include "spatial_index.h"
#include "graph_algo.h"
#include "graph_algo_adapters.h"

struct graph_services_t
{
  graph_services_t(graph_t const * graph, spatial_index_t const * index);
  
  graph_t const * graph() const;
  spatial_index_t const * spatial_index() const;
  
  //OutIter should be an output iterator, accepting vertex_t
  template<class OutIter>
  double find_shortest_path(wgs82_point_t const & source, wgs82_point_t const & destination, OutIter output_iter) const;
  
  // OutIter should be an output iterator, accepting edge_t
  template<class OutIter>
  void mark_nearest_edges(wgs82_point_t const & point, OutIter output_iter) const;
private:
  graph_t const * graph_;
  spatial_index_t const * index_;
};


// Inline members
inline graph_services_t::graph_services_t(graph_t const * graph, spatial_index_t const * index)
  : graph_(graph), index_(index)
{
}

inline graph_t const * graph_services_t::graph() const
{
  return graph_;
}

inline spatial_index_t const * graph_services_t::spatial_index() const
{
  return index_;
}

// Implementation

namespace details
{
template<class DijkstraAdapter>
struct adapter_inserter_f
{
  adapter_inserter_f(graph_t const & graph, DijkstraAdapter & adapter, vertex_t pos, bool forward)
    : graph_(graph), adapter_(adapter), pos_(pos), forward_(forward)
  {    
  }
  
  void operator () (edge_t edge)
  {
    vertex_t source = graph_.get_vertex_by_id(edge.source());
    vertex_t destination = graph_.get_vertex_by_id(edge.destination());
    
    std::pair<double, vertex_data_t> dist = get_distance_to_edge(pos_.data(), source.data(), destination.data());
    if (dist.first > 60.0)
      return;
    
    vertex_t v(INVALID_VERTEX_ID, dist.second);
     
    adapter_.decrease_key(v, dist.first, pos_);
    if (!forward_)
    {
      adapter_.decrease_key(source, dist.first + get_distance(v.data(), source.data()), v);
    }
    else
    {
      adapter_.decrease_key(destination, dist.first + get_distance(v.data(), destination.data()), v);
    }
  }
  
private:
  graph_t const & graph_;
  DijkstraAdapter & adapter_;
  vertex_t pos_;    
  bool forward_;
};
} // namespace details

template<class OutIter>
double graph_services_t::find_shortest_path(wgs82_point_t const & source, wgs82_point_t const & destination, OutIter output_iter) const
{
  details::dijkstra_graph_t adapter(*graph_);
  details::dijkstra_graph_t adapter_2(*graph_);
  
  vertex_t source_vertex(INVALID_VERTEX_ID, source);
  vertex_t destination_vertex(INVALID_VERTEX_ID, destination);
  adapter.decrease_key(source_vertex, 0.0, source_vertex);
  adapter_2.decrease_key(destination_vertex, 0.0, destination_vertex);

  index_->for_all_nearest_edges(source_vertex.data(), 
                                details::adapter_inserter_f<details::dijkstra_graph_t>(*graph_, 
                                                                                       adapter, 
                                                                                       source_vertex, 
                                                                                       true));  
  index_->for_all_nearest_edges(destination_vertex.data(), 
                                details::adapter_inserter_f<details::dijkstra_graph_t>(*graph_, 
                                                                                       adapter_2, 
                                                                                       destination_vertex, 
                                                                                       false));  

  return base_bidirectional_dijkstra(adapter, adapter_2, source_vertex, destination_vertex, output_iter);      
}

template<class OutIter>
void graph_services_t::mark_nearest_edges(wgs82_point_t const & point, OutIter output_iter) const
{
  typedef OutIter & (OutIter::*eq_type)(edge_t const &);
  eq_type tmp = &OutIter::operator=;
  index_->for_all_nearest_edges(point, boost::bind(tmp, output_iter, _1));
}
