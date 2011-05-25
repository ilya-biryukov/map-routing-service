#pragma once

#include <vector>
#include <fstream>
#include <boost/noncopyable.hpp>
#include <boost/unordered_map.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "graph_data.h"
#include "raw_graph_data.h"

namespace details
{
struct mutable_edge_t;
typedef std::vector<mutable_edge_t> edges_list_t;

struct mutable_edge_t
{
  mutable_edge_t(size_t dst, double weight)
    : dst(dst), weight(weight)
  {
  }
  size_t dst;    
  double weight;
};
  
struct edges_list_iterator 
  : boost::iterator_facade<edges_list_iterator, // CRTP
      edge_t, // value type
      boost::bidirectional_traversal_tag, // traversal category
      edge_t>
{
  edges_list_iterator(vertex_id_t src_id, edges_list_t::const_iterator iter);
private:    
  edge_t dereference();    
  void increment();
  void decrement();
  
  vertex_id_t src_vertex_id_;
  edges_list_t::const_iterator edges_it_;
  
  friend class boost::iterator_core_access;
};
} // namespace details

struct mutable_graph_t : boost::noncopyable
{
  typedef boost::iterator_range<details::edges_list_iterator> edges_range_t;

  mutable_graph_t();
  
//   edges_range_t get_incident_edges(vertex_t const & v, edge_type_t edges_type) const;
  size_t get_vertices_count() const;  
  vertex_t get_vertex_by_id(vertex_id_t id) const;
  
  vertex_t add_vertex(vertex_data_t const & data);
  void add_edge(vertex_t const & src, vertex_t const & dst, double weight);  
  
  
  // Note: Clears all the graph data after completed
  void dump_to_file(std::string const & filename);
private:  
  typedef details::edges_list_t edges_list_t;
  typedef std::pair<edges_list_t, edges_list_t> edges_lists_t;
  typedef std::vector<vertex_t> vertices_cont_t;
  typedef std::vector<edges_lists_t> edges_lists_cont_t;

  size_t update_indicies();
  void clear();
    
  vertices_cont_t vertices_;
  edges_lists_cont_t edges_lists_;
  vertex_id_t last_id_;
  size_t edges_count_;

  friend struct cont_value_id_comparator_f;
  friend struct mutable_edge_t;
};


// Implementation
struct vertex_t_id_comparator_f
{
  bool operator () (vertex_t const & lhs, vertex_t const & rhs)
  {
    return lhs.id() < rhs.id();
  }
};

namespace details
{
inline edges_list_iterator::edges_list_iterator(vertex_id_t src_id, edges_list_t::const_iterator iter)
  : src_vertex_id_(src_id), edges_it_(iter)
{
}

  
inline void edges_list_iterator::decrement()
{
  --edges_it_;  
}

inline void edges_list_iterator::increment()
{
  ++edges_it_;
}

inline edge_t edges_list_iterator::dereference()
{
  return edge_t(src_vertex_id_,  edges_it_->dst, edges_it_->weight);
}
} // namespace details

inline mutable_graph_t::mutable_graph_t()
  : last_id_(0), edges_count_(0)
{
}

// inline mutable_graph_t::edges_range_t mutable_graph_t::get_incident_edges(vertex_t const & v, 
//                                                                           edge_type_t edges_type) const
// {
//   if (v.id < vertices_.size())  
//   {
//       edges_list_t const * e_lst;
//       if (edges_type == EDGE_NORMAL)
//         e_lst = &vertices_[v.id].second.first;
//       else 
//         e_lst = &vertices_[v.id].second.second;
//     
// 
//     return edges_range_t(details::edges_list_iterator(v.id, e_lst->begin()), 
//                          details::edges_list_iterator(v.id, e_lst->end()));
//   }
//   else
//   {
//     return edges_range_t(details::edges_list_iterator(v.id, edges_list_t::iterator()),
//                          details::edges_list_iterator(v.id, edges_list_t::iterator()));
//   }
// }

inline size_t mutable_graph_t::get_vertices_count() const
{
  return vertices_.size();  
}

inline vertex_t mutable_graph_t::get_vertex_by_id(vertex_id_t id) const
{  
  vertices_cont_t::const_iterator it = std::lower_bound(vertices_.begin(),
                                                        vertices_.end(),
                                                        vertex_t(id, vertex_data_t(0.0, 0.0)),
                                                        vertex_t_id_comparator_f());
  if (it == vertices_.end())
    return vertex_t();
  else
    return *it;
}

inline vertex_t mutable_graph_t::add_vertex(vertex_data_t const & data)
{
  vertex_t v(last_id_++, data);
  vertices_.push_back(v);  
  edges_lists_.push_back(edges_lists_t());
  return v;
}

inline void mutable_graph_t::add_edge(vertex_t const & src, 
                                      vertex_t const & dst, 
                                      double weight)
{
  assert(src.id != dst.id);
  edges_lists_[src.id()].first.push_back(details::mutable_edge_t(dst.id(), weight));
  edges_lists_[dst.id()].second.push_back(details::mutable_edge_t(src.id(), weight));
  edges_count_ += 2;
}

inline size_t mutable_graph_t::update_indicies()
{
  size_t i = 0;
  edges_lists_cont_t::iterator elit = edges_lists_.begin();
  vertices_cont_t::iterator vit = vertices_.begin();
  for (; vit != vertices_.end(); ++vit, ++elit)
  {
    if (elit->first.empty() && elit->second.empty())
    {      
      vit->set_id(INVALID_VERTEX_ID);
    }
    else
    {
      vit->set_id(i++);      
    }
  }
  
  return i;
}

inline void mutable_graph_t::dump_to_file(std::string const & filename)
{
  namespace boostfs = boost::filesystem3;
  namespace boostinterp = boost::interprocess;  
  
  if (!boostfs::exists(filename))
  {
    std::ofstream file(filename.c_str());
  }
  
  size_t vertices_count = update_indicies();
  size_t size_needed = sizeof(details::raw_vertices_header_t)  
    + sizeof(details::raw_vertex_t) * vertices_count
    + sizeof(details::raw_edges_header_t)
    + sizeof(details::raw_edge_t) * edges_count_ ;
  boostfs::resize_file(filename, size_needed);
  
  boostinterp::file_mapping mapping(filename.c_str(), boostinterp::read_write);
  boostinterp::mapped_region region(mapping, boostinterp::read_write, 0, size_needed);
  
  details::raw_vertices_header_t * vertices_header = static_cast<details::raw_vertices_header_t *>(region.get_address());  
  vertices_header->vertices_count = vertices_count;
  
  details::raw_edges_header_t * edges_header = reinterpret_cast<details::raw_edges_header_t *>(&vertices_header->vertices[vertices_count]);  
  edges_header->edges_count = edges_count_;
  
  size_t next_edge = 0;  
  size_t i = 0;
  vertices_cont_t::const_iterator vit = vertices_.begin();
  edges_lists_cont_t::const_iterator elit = edges_lists_.begin();
  for (; vit!= vertices_.end();
       ++vit, ++elit)
  {
    if (!vit->is_valid())
    {
      continue;
    }
    
    vertices_header->vertices[i].lat = vit->data().lat();
    vertices_header->vertices[i].lon = vit->data().lon();
    
    if (elit->first.empty())
    {
      vertices_header->vertices[i].edges_begin_id = details::INVALID_RAW_EDGE_ID;
      vertices_header->vertices[i].edges_end_id = details::INVALID_RAW_EDGE_ID;
    }
    else
    {
      vertices_header->vertices[i].edges_begin_id = next_edge;
      vertices_header->vertices[i].edges_end_id = next_edge + elit->first.size();
      
      for (edges_list_t::const_iterator eit = elit->first.begin();
           eit != elit->first.end();
           ++eit)
      {
        edges_header->edges[next_edge].destination = vertices_[eit->dst].id();
        edges_header->edges[next_edge].weight = eit->weight;
        ++next_edge;
      }
    }
    if (elit->second.empty())
    {      
      vertices_header->vertices[i].transposed_edges_begin_id = details::INVALID_RAW_EDGE_ID;
      vertices_header->vertices[i].transposed_edges_end_id = details::INVALID_RAW_EDGE_ID;
    }
    else
    {      
      vertices_header->vertices[i].transposed_edges_begin_id = next_edge;
      vertices_header->vertices[i].transposed_edges_end_id = next_edge + elit->second.size();
      for (edges_list_t::const_iterator eit = elit->second.begin();
           eit != elit->second.end();
           ++eit)
      {
        edges_header->edges[next_edge].destination = vertices_[eit->dst].id();
        edges_header->edges[next_edge].weight = eit->weight;
        ++next_edge;
      }    
    }
 
    ++i;
  }  
  
  assert(next_edge == edges_count_);
  assert(i == vertices_count);
  
  clear();
}

inline void mutable_graph_t::clear()
{
  vertices_.clear();
  edges_lists_.clear();
  last_id_ = 0;
  edges_count_ = 0;
}
