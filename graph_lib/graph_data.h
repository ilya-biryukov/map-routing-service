#pragma once

#include <boost/cstdint.hpp>
#include <boost/functional/hash.hpp>

#include "map_geometry.h"
#include "vector_geometry.h"

/**
 * @brief Type for vertex identifier
 */
typedef boost::uint64_t vertex_id_t;
/**
 * @brief Type for vertex data
 */
typedef wgs82_point_t vertex_data_t;

/**
 * @brief Type of the edge.
 *      EDGE_NORMAL is the type of the edges of the graph
 *      EDGE_REVERSED is the type of the edges of reversed graph
 */
enum edge_type_t
{
  EDGE_NORMAL,
  EDGE_REVERSED
};

vertex_id_t const INVALID_VERTEX_ID = -1L;

/**
 * @brief Graph vertex
 */
struct vertex_t
{
  vertex_t(vertex_id_t id, vertex_data_t const & data);
  /**
   * @brief Creates vertex with invalid id
   */
  vertex_t();
  
  bool is_valid() const;  
  
  vertex_id_t id() const;
  void        set_id(vertex_id_t id);  
  
  vertex_data_t data() const;
  void          set_data(vertex_data_t const & data);
  
private:
  vertex_id_t id_;
  vertex_data_t data_;
};


bool operator == (vertex_t const & lhs, vertex_t const & rhs);
bool operator != (vertex_t const & lhs, vertex_t const & rhs);
size_t hash_value(vertex_t const & val);

/**
 * @brief Edge of the graph.
 */
struct edge_t
{
  /**
   * @brief Creates edge with invalid id.
   */
  edge_t();
  edge_t(vertex_id_t source, vertex_id_t destination, double weight);
  
  bool is_valid() const;  
  
  vertex_id_t source() const;
  void        set_source(vertex_id_t source);
  
  vertex_id_t destination() const;
  void        set_destination(vertex_id_t destination);
  
  double weight() const;
  void   set_weight(double weight);  

private:  
  vertex_id_t source_;
  vertex_id_t destination_;
  double weight_;
};


/**
  * @brief Computes distance from point pos to the edge starting at edge_start and ending at edge_end
  * @param edge_start Point where the edge starts
  * @param pos Point for computing distance
  * @param edge_end Point where the edge ends
  * @return A pair of distance to the edge and closest to pos point on the edge
  */
std::pair<double, wgs82_point_t> get_distance_to_edge(wgs82_point_t pos, wgs82_point_t edge_start, wgs82_point_t edge_end);

// Implementation
inline vertex_t::vertex_t(vertex_id_t id, vertex_data_t const & data) :
  id_(id), data_(data)
{
}

inline vertex_t::vertex_t() 
  : id_(INVALID_VERTEX_ID), data_(0.0, 0.0)
{
}

inline bool vertex_t::is_valid() const
{
  return id_ != INVALID_VERTEX_ID;
}
  
inline vertex_id_t vertex_t::id() const
{
  return id_;
}

inline void vertex_t::set_id(vertex_id_t id)
{
  id_ = id;
}
  
inline vertex_data_t vertex_t::data() const
{
  return data_;
}

inline void vertex_t::set_data(vertex_data_t const & data)
{
  data_ = data;
}

inline bool operator == (vertex_t const & lhs, vertex_t const & rhs)
{
  if (lhs.is_valid() && rhs.is_valid())
    return lhs.id() == rhs.id();
  else
    return lhs.data() == rhs.data();
}

inline bool operator != (vertex_t const & lhs, vertex_t const & rhs)
{
  return !(lhs == rhs);
}

inline size_t hash_value(vertex_t const & val)
{
  boost::hash<int> hasher;
  return hasher(val.id());
}

inline edge_t::edge_t(vertex_id_t source, vertex_id_t destination, double weight) 
  : source_(source), destination_(destination), weight_(weight)
{
}

inline edge_t::edge_t() 
  : source_(INVALID_VERTEX_ID), destination_(INVALID_VERTEX_ID)
{
}

inline bool edge_t::is_valid() const
{
  return source() != INVALID_VERTEX_ID && destination() != INVALID_VERTEX_ID;
}

inline vertex_id_t edge_t::source() const
{
  return source_;
}

inline void edge_t::set_source(vertex_id_t source)
{
  source_ = source;
}

inline vertex_id_t edge_t::destination() const
{
  return destination_;
}

inline void edge_t::set_destination(vertex_id_t destination)
{
  destination_ = destination;
}

inline double edge_t::weight() const
{
  return weight_;
}

inline void edge_t::set_weight(double weight)
{
  weight_ = weight;
}

inline std::pair<double, wgs82_point_t> get_distance_to_edge(wgs82_point_t pos, wgs82_point_t edge_start, wgs82_point_t edge_end)
{
  mercator_point_t start(edge_start);
  mercator_point_t end(edge_end);
  mercator_point_t point(pos);
  
  vector2_t e(end.x() - start.x(), end.y() - start.y());
  vector2_t a(point.x() - start.x(), point.y() - start.y());
    
  double proj = scalar_product(e, a) / e.length();
  
  if (proj <= 0.0)
  {
    return std::make_pair(get_distance(pos, edge_start), edge_start);    
  }
  else if (proj >= e.length())
  {
    return std::make_pair(get_distance(pos, edge_end), edge_end);
  }
  else
  {
    e *= proj / e.length();

    wgs82_point_t projection_point(mercator_point_t(e.x() + start.x(), e.y() + start.y()));

    return std::make_pair(get_distance(projection_point, pos), projection_point);
  }
}
