#pragma once

#include <functional>

#include "graph.h"
#include "graph_data.h"
#include "binary_heap.h"

namespace details
{
struct dijkstra_vertex_t
{  
  dijkstra_vertex_t(vertex_t v, heap_id h, double dist, vertex_t pred)
      : vertex(v), processed(false), v_heap_id(h), dist(dist), pred(pred)
  {
  }

  vertex_t vertex;
  bool processed;
  heap_id v_heap_id;
  double dist;
  vertex_t pred; 
};


struct dijkstra_graph_t
{
  dijkstra_graph_t(graph_t const & graph);
  
  std::pair<double, vertex_t> next() const;  
  void dequeue();
  void decrease_key(vertex_t v, double new_dist, vertex_t new_pred);
  bool empty() const;
  vertex_t get_predecessor(vertex_t v) const;
  double get_dist(vertex_t v) const;
  graph_t const & get_graph() const;
  bool was_processed(vertex_t v) const;
private:
  typedef boost::unordered_map<vertex_t, dijkstra_vertex_t> map_t;
  
  graph_t const & graph_;
  binary_heap_t<double, vertex_t> heap_;
  map_t info_map_;
};
} // namespace details