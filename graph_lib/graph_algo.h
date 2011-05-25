#pragma once

#include <iostream>
#include <functional>
#include <boost/unordered_map.hpp>

#include "binary_heap.h"
#include "graph.h"
#include "graph_data.h"
#include "graph_algo_adapters.h"
#include "graph_algo_potentials.h"

#define GRAPH_DEBUG_STATS

template<class OutIter, class DijkstraAdapter>
double dijkstra(graph_t const & g, vertex_t v1, vertex_t v2, OutIter out);

template<class OutIter, class DijkstraAdapter>
double bidirectional_dijkstra(graph_t const & g,
                              vertex_t v1, 
                              vertex_t v2, 
                              OutIter out);

template<class OutIter, class DijkstraAdapter>
double a_star(graph_t const & g,
              vertex_t v1, 
              vertex_t v2, 
              OutIter out);

template<class OutIter, class DijkstraAdapter>
double bidirectional_a_star(graph_t const & g,
                            vertex_t v1, 
                            vertex_t v2, 
                            OutIter out);

template<class OutIter, class Potential, class DijkstraAdapter>
double a_star_custom_potential(graph_t const & g,
                               vertex_t v1, 
                               vertex_t v2, 
                               OutIter out, 
                               Potential p = Potential());

template<class OutIter, class Potential, class DijkstraAdapter>
double bidirectional_a_star_custom_potential(graph_t const & g,
                                             vertex_t v1, 
                                             vertex_t v2, 
                                             OutIter out, 
                                             Potential p = Potential());



template<class DijkstraAdapter, class OutIter>
double base_dijkstra(DijkstraAdapter & a, 
                     vertex_t v1, 
                     vertex_t v2, 
                     OutIter out);

template<class DijkstraAdapter, class OutIter>
double base_bidirectional_dijkstra(DijkstraAdapter & forward_dijkstra, 
                                   DijkstraAdapter & reverse_dijkstra,
                                   vertex_t v1, 
                                   vertex_t v2, 
                                   OutIter out);

template<class DijkstraAdapter, class OutIter, class Potential>
double base_a_star(DijkstraAdapter & a,
                   vertex_t v1, 
                   vertex_t v2, 
                   OutIter out, 
                   Potential p = Potential());

template<class DijkstraAdapter, class OutIter, class Potential>
double base_bidirectional_a_star(DijkstraAdapter & forward_dijkstra,
                                 DijkstraAdapter & reverse_dijkstra,
                                 vertex_t v1, 
                                 vertex_t v2, 
                                 OutIter out, 
                                 Potential p = Potential());


// Implementation

template<class DijkstraAdapter, class OutIter>
double base_dijkstra(DijkstraAdapter & a, 
                     vertex_t v1, 
                     vertex_t v2, 
                     OutIter out)
{  
#ifdef GRAPH_DEBUG_STATS
  size_t edges_visited = 0;
#endif
  graph_t const & g = a.get_graph();
  while (!a.empty())
  {
    std::pair<double, vertex_t> current_vertex = a.next();
    a.dequeue();
    if (current_vertex.second == v2)
      break;

    graph_t::edges_range adj = g.get_incident_edges(current_vertex.second, EDGE_NORMAL);

    for (graph_t::edges_range::iterator it = adj.begin(); it != adj.end(); ++it)
    {
#ifdef GRAPH_DEBUG_STATS
      ++edges_visited;
#endif
      vertex_t dest = g.get_vertex_by_id(it->destination());
      
      double new_dist = it->weight() + current_vertex.first;
      assert(new_dist >= 0.0);
      
      if (new_dist < a.get_dist(dest))
      {
        a.decrease_key(dest, new_dist, current_vertex.second);
      }
    }
  }

  if (a.was_processed(v2))
  {
    vertex_t cur_v = v2;
    while (cur_v != v1)
    {
      *(out++) = cur_v;
      cur_v = a.get_predecessor(cur_v);
    }
    *out = v1;
  }
  
#ifdef GRAPH_DEBUG_STATS
  std::cerr << "Dijkstra done. Edges visited:" << edges_visited << std::endl;
#endif

  return a.get_dist(v2);
}

namespace details
{ 
template <class DijkstraAdapter>
bool make_bidi_search_step(graph_t const & g, 
                           DijkstraAdapter & primary_search, 
                           DijkstraAdapter & secondary_search, 
                           bool forward,
                           vertex_t * best_rally_point, 
                           double * best_dist
#ifdef GRAPH_DEBUG_STATS
                           ,size_t & edges_visited
#endif                            
)
{
  std::pair<double, vertex_t> v = primary_search.next();
  primary_search.dequeue();
  if (secondary_search.was_processed(v.second))
    return false;

  edge_type_t type = forward ? EDGE_NORMAL : EDGE_REVERSED;
  graph_t::edges_range adj = g.get_incident_edges(v.second, type);
  for (graph_t::edges_range::iterator it = adj.begin(); it != adj.end(); ++it)
  {
#ifdef GRAPH_DEBUG_STATS
    ++edges_visited;
#endif
    double new_dist = it->weight() + v.first;
    assert(new_dist >= 0.0);

    vertex_t dest = g.get_vertex_by_id(it->destination());

    if (new_dist < primary_search.get_dist(dest))
    {
      primary_search.decrease_key(dest, new_dist, v.second);
      if (secondary_search.was_processed(dest))
      {
        double new_total_dist = secondary_search.get_dist(dest) + new_dist;
        if (new_total_dist < *best_dist)
        {
          *best_dist = new_total_dist;
          *best_rally_point = dest;
        }
      }
    }
  }
  return true;
}
} // namespace details

template<class DijkstraAdapter, class OutIter>
double base_bidirectional_dijkstra(DijkstraAdapter & forward_dijkstra, 
                                   DijkstraAdapter & reverse_dijkstra,
                                   vertex_t v1, 
                                   vertex_t v2, 
                                   OutIter out)
{
#ifdef GRAPH_DEBUG_STATS
  size_t edges_visited = 0;
#endif
  graph_t const & g = forward_dijkstra.get_graph();
  
  vertex_t best_rally_point;
  double best_dist = std::numeric_limits<double>::max();
  
  while (!forward_dijkstra.empty() && !reverse_dijkstra.empty())
  {
    if (!details::make_bidi_search_step(g, 
                                        forward_dijkstra, 
                                        reverse_dijkstra, 
                                        true, 
                                        &best_rally_point, 
                                        &best_dist
#ifdef GRAPH_DEBUG_STATS
                                        ,edges_visited
#endif
                                       ) 
        || !details::make_bidi_search_step(g, 
                                        reverse_dijkstra, 
                                        forward_dijkstra,
                                        false,
                                        &best_rally_point,
                                        &best_dist
#ifdef GRAPH_DEBUG_STATS
                                        ,edges_visited
#endif
                                          ))
    {
      break;
    }
  }
  
  if (best_rally_point.is_valid())
  {
    std::vector<vertex_t> path_to_v2;
    vertex_t v = best_rally_point;
    while (v != v2)
    {
      path_to_v2.push_back(v);
      v = reverse_dijkstra.get_predecessor(v);
    }
    *(out++) = v2;
    while (!path_to_v2.empty())
    {
      *(out++) = path_to_v2.back();
      path_to_v2.pop_back();
    }
    
    if (best_rally_point != v1)
    {
      v = forward_dijkstra.get_predecessor(best_rally_point);
      while (v != v1)
      {
        *(out++) = v;
        v = forward_dijkstra.get_predecessor(v);
      }
      *(out++) = v1;
    }    
  }
  
#ifdef GRAPH_DEBUG_STATS
  std::cerr << "Bidirectional dijkstra done. Edges visited: " << edges_visited << std::endl;
#endif  
  return best_dist;
}

template<class DijkstraAdapter, class OutIter, class Potential>
double base_a_star(DijkstraAdapter & a,
                   vertex_t v1, 
                   vertex_t v2, 
                   OutIter out, 
                   Potential p = Potential())
{    
#ifdef GRAPH_DEBUG_STATS
  size_t edges_visited = 0;
#endif
  graph_t const & g = a.get_graph();
  
  a.decrease_key(v1, p(v1, v2), v1);
  while (!a.empty())
  {
    std::pair<double, vertex_t> current_vertex = a.next();
    a.dequeue();
    if (current_vertex.second == v2)
      break;

    graph_t::edges_range adj = g.get_incident_edges(current_vertex.second, EDGE_NORMAL);

    for (graph_t::edges_range::iterator it = adj.begin(); it != adj.end(); ++it)
    {
#ifdef GRAPH_DEBUG_STATS
      ++edges_visited;
#endif
      vertex_t dest = g.get_vertex_by_id(it->destination());
      double new_dist = current_vertex.first + it->weight() + p(v2, dest) - p(v2, current_vertex.second);
      assert(new_dist >= 0.0);

      if (new_dist < a.get_dist(dest))
      {
        a.decrease_key(dest, new_dist, current_vertex.second);
      }
    }
  }

  if (a.was_processed(v2))
  {
    vertex_t cur_v = v2;
    while (cur_v != v1)
    {
      *(out++) = cur_v;
      cur_v = a.get_predecessor(cur_v);
    }
    *out = v1;
  }
  
#ifdef GRAPH_DEBUG_STATS
  std::cerr << "Astar done. Edges visited:" << edges_visited << std::endl;
#endif

  return a.get_dist(v2) - p(v2, v2);
}

namespace details
{ 
template<class Potential>
double bidi_astar_heuristics(vertex_t const & v1, // source
                             vertex_t const & v2, // destination
                             vertex_t const & v,  
                             bool forward,
                             Potential p = Potential())
{
  double h = (p(v2, v) - p(v1, v)) / 2.0;
  return forward ? h : -h;
}
  
template<class DijkstraAdapter, class Potential>
bool make_bidi_astar_search_step(graph_t const & g, 
                                 DijkstraAdapter & primary_search, 
                                 DijkstraAdapter & secondary_search, 
                                 vertex_t const & v1,
                                 vertex_t const & v2,
                                 bool forward,
                                 vertex_t * best_rally_point, 
                                 double * best_dist,
#ifdef GRAPH_DEBUG_STATS
                                 size_t & edges_visited,
#endif                  
                                 Potential p = Potential()
)
{
  std::pair<double, vertex_t> v = primary_search.next();
  primary_search.dequeue();
  if (secondary_search.was_processed(v.second))
    return false;
  
  double v_heuristics = bidi_astar_heuristics(v1, v2, v.second, forward, p);

  edge_type_t type = forward ? EDGE_NORMAL : EDGE_REVERSED;
  graph_t::edges_range adj = g.get_incident_edges(v.second, type);
  for (graph_t::edges_range::iterator it = adj.begin(); it != adj.end(); ++it)
  {
#ifdef GRAPH_DEBUG_STATS
    ++edges_visited;
#endif
    vertex_t dest = g.get_vertex_by_id(it->destination());

    double dest_heuristics = bidi_astar_heuristics(v1, v2, dest, forward, p);
    double new_dist = it->weight() + v.first + dest_heuristics - v_heuristics;
    assert(new_dist >= 0.0);

    if (new_dist < primary_search.get_dist(dest))
    {
      primary_search.decrease_key(dest, new_dist, v.second);
      if (secondary_search.was_processed(dest))
      {
        double new_total_dist = secondary_search.get_dist(dest) + new_dist;
        if (new_total_dist < *best_dist)
        {
          *best_dist = new_total_dist;
          *best_rally_point = dest;
        }
      }
    }
  }
  return true;
}
} // namespace details

template<class DijkstraAdapter, class OutIter, class Potential>
double base_bidirectional_a_star(DijkstraAdapter & forward_dijkstra,
                                 DijkstraAdapter & reverse_dijkstra,
                                 vertex_t v1, 
                                 vertex_t v2, 
                                 OutIter out, 
                                 Potential p = Potential())
{
#ifdef GRAPH_DEBUG_STATS
  size_t edges_visited = 0;
#endif
  graph_t const & g = forward_dijkstra.get_graph();
  
  vertex_t best_rally_point;
  double best_dist = std::numeric_limits<double>::max();

  forward_dijkstra.decrease_key(v1, details::bidi_astar_heuristics(v1, v2, v1, true, p), v1);
  reverse_dijkstra.decrease_key(v2, details::bidi_astar_heuristics(v1, v2, v2, false, p), v2);

  while (!forward_dijkstra.empty() && !reverse_dijkstra.empty())
  {
    if (!details::make_bidi_astar_search_step(g, 
                                              forward_dijkstra, 
                                              reverse_dijkstra, 
                                              v1,
                                              v2,
                                              true, 
                                              &best_rally_point, 
                                              &best_dist,
#ifdef GRAPH_DEBUG_STATS
                                              edges_visited,
#endif
                                              p
                                       ) 
        || !details::make_bidi_astar_search_step(g, 
                                                 reverse_dijkstra, 
                                                 forward_dijkstra,
                                                 v1,
                                                 v2,
                                                 false,
                                                 &best_rally_point,
                                                 &best_dist,
#ifdef GRAPH_DEBUG_STATS
                                                 edges_visited,
#endif
                                                 p))
    {
      break;
    }
  }
  
  if (best_rally_point.is_valid())
  {
    std::vector<vertex_t> path_to_v2;
    vertex_t v = best_rally_point;
    while (v != v2)
    {
      path_to_v2.push_back(v);
      v = reverse_dijkstra.get_predecessor(v);
    }
    *(out++) = v2;
    while (!path_to_v2.empty())
    {
      *(out++) = path_to_v2.back();
      path_to_v2.pop_back();
    }
    
    if (best_rally_point != v1)
    {
      v = forward_dijkstra.get_predecessor(best_rally_point);
      while (v != v1)
      {
        *(out++) = v;
        v = forward_dijkstra.get_predecessor(v);
      }
      *(out++) = v1;
    }    
  }
  
#ifdef GRAPH_DEBUG_STATS
  std::cerr << "Bidirectional astar done. Edges visited: " << edges_visited << std::endl;
#endif  
  return best_dist - details::bidi_astar_heuristics(v1, v2, best_rally_point, true, p) 
                   - details::bidi_astar_heuristics(v1, v2, best_rally_point, false, p);
}

template<class OutIter, class DijkstraAdapter>
double dijkstra(graph_t const & g, vertex_t v1, vertex_t v2, OutIter out)
{
  DijkstraAdapter adapter(g);
  adapter.decrease_key(v1, 0.0, v1);
  return base_dijkstra(adapter, v1, v2, out);
}


template<class OutIter, class DijkstraAdapter>
double bidirectional_dijkstra(graph_t const & g,
                              vertex_t v1, 
                              vertex_t v2, 
                              OutIter out)
{
  DijkstraAdapter adapter_forward(g);
  DijkstraAdapter adapter_reversed(g);
  
  adapter_forward.decrease_key(v1, 0.0, v1);
  adapter_reversed.decrease_key(v2, 0.0, v2);
  
  return base_bidirectional_dijkstra(adapter_forward, adapter_reversed, v1, v2, out);
}

template<class OutIter, class Potential, class DijkstraAdapter>
double a_star_custom_potential(graph_t const & g,
                               vertex_t v1, 
                               vertex_t v2, 
                               OutIter out, 
                               Potential p = Potential())
{
  DijkstraAdapter adapter(g);
  
  adapter.decrease_key(v1, p(v1, v2), v1);
  
  return base_a_star(adapter, v1, v2, out, p);
}

template<class OutIter, class DijkstraAdapter>
double a_star(graph_t const & g,
              vertex_t v1, 
              vertex_t v2, 
              OutIter out)
{
  return a_star_custom_potential<OutIter, details::potential_dist_f, DijkstraAdapter>
    (g, v1, v2, out, details::potential_dist_f());
}

template<class OutIter, class Potential, class DijkstraAdapter>
double bidirectional_a_star_custom_potential(graph_t const & g,
                                             vertex_t v1, 
                                             vertex_t v2, 
                                             OutIter out, 
                                             Potential p = Potential())
{
  DijkstraAdapter adapter_forward(g);
  DijkstraAdapter adapter_reversed(g);
  
  adapter_forward.decrease_key(v1, details::bidi_astar_heuristics(v1, v2, v1, true, p), v1);
  adapter_reversed.decrease_key(v2, details::bidi_astar_heuristics(v1, v2, v1, false, p), v2);
  
  return base_bidirectional_a_star(adapter_forward, adapter_reversed, v1, v2, out, p);
}

template<class OutIter, class DijkstraAdapter>
double bidirectional_a_star(graph_t const & g,
                            vertex_t v1, 
                            vertex_t v2, 
                            OutIter out)
{
  return bidirectional_a_star_custom_potential<OutIter, details::potential_dist_f, DijkstraAdapter>
    (g, v1, v2, out, details::potential_dist_f());
}