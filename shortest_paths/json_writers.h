#pragma once

#include <libjson/libjson.h>

#include "graph.h"
#include "graph_data.h"


struct json_output_iter_t
{
  json_output_iter_t(JSONNode * node, graph_t const * graph);
  
  json_output_iter_t & operator = (json_output_iter_t const & rhs);
  
  json_output_iter_t & operator = (edge_t const & edge);
  json_output_iter_t & operator = (vertex_t const & vertex);
  
  json_output_iter_t operator ++ ();
  json_output_iter_t operator ++ (int);  
  
  json_output_iter_t & operator * ();

private:  
  JSONNode * node_;
  graph_t const * graph_;
};

// Inline implementation

namespace details
{
  JSONNode geodata_to_json(wgs82_point_t const & data)
  {
    JSONNode node(JSON_ARRAY);
    
    node.push_back(JSONNode("", data.lon()));
    node.push_back(JSONNode("", data.lat()));
    
    return node;
  }
}

json_output_iter_t::json_output_iter_t(JSONNode * node, graph_t const * graph)
  : node_(node), graph_(graph)
{
}

json_output_iter_t & json_output_iter_t::operator = (json_output_iter_t const & rhs)
{
  node_ = rhs.node_;
  graph_ = rhs.graph_;
  return *this;
}

json_output_iter_t & json_output_iter_t::operator = (edge_t const & edge)
{
  vertex_t src(graph_->get_vertex_by_id(edge.source()));
  vertex_t dst(graph_->get_vertex_by_id(edge.destination()));
  
  JSONNode edge_node(JSON_ARRAY);
  edge_node.push_back(details::geodata_to_json(src.data()));
  edge_node.push_back(details::geodata_to_json(dst.data()));
  node_->push_back(edge_node);
  
  return *this;
}

json_output_iter_t & json_output_iter_t::operator = (vertex_t const & vertex)
{
  node_->push_back(details::geodata_to_json(vertex.data()));
  return *this;
}

json_output_iter_t json_output_iter_t::operator ++ ()
{
  return *this;
}

json_output_iter_t json_output_iter_t::operator ++ (int)
{
  return *this;
}


json_output_iter_t & json_output_iter_t::operator * ()
{
  return *this;
}

