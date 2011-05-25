#include "graph_algo_adapters.h"

namespace details
{
dijkstra_graph_t::dijkstra_graph_t(graph_t const & graph)
    : graph_(graph)
{
}

std::pair<double, vertex_t> dijkstra_graph_t::next() const
{
  return heap_.top();
}

void dijkstra_graph_t::dequeue()
{    
  std::pair<double, vertex_t> tmp = heap_.top();    
  info_map_.find(tmp.second)->second.processed = true;
  
  heap_.extract_top();
}

void dijkstra_graph_t::decrease_key(vertex_t v, double new_dist, vertex_t new_pred)
{
  map_t::iterator it = info_map_.find(v);
  if (it == info_map_.end())
  {
    info_map_.insert(
      std::make_pair(
        v,
        dijkstra_vertex_t(v,
                        heap_.push(new_dist, v),
                        new_dist, new_pred)));
  }
  else
  {
    heap_.decrease_key(it->second.v_heap_id, new_dist);
    it->second.dist = new_dist;
    it->second.pred = new_pred;
  }
}

bool dijkstra_graph_t::empty() const
{
  return heap_.empty();
}

vertex_t dijkstra_graph_t::get_predecessor(vertex_t v) const
{
  return info_map_.find(v)->second.pred;
}

double dijkstra_graph_t::get_dist(vertex_t v) const
{
  map_t::const_iterator it = info_map_.find(v);
  if (it == info_map_.end())
    return std::numeric_limits<double>::max();
  else
    return it->second.dist;
}

graph_t const & dijkstra_graph_t::get_graph() const
{
  return graph_;
}

bool dijkstra_graph_t::was_processed(vertex_t v) const
{
  map_t::const_iterator it = info_map_.find(v);
  return (it != info_map_.end() && it->second.processed);
}
} // namespace details