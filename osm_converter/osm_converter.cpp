#include "osm_converter.h"

#include <istream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/unordered_map.hpp>
#include <boost/cstdint.hpp>
#include <boost/scoped_array.hpp>
#include <boost/lexical_cast.hpp>

#include <expat.h>

#include "mutable_graph.h"
#include "graph_data.h"

namespace
{
typedef boost::uint32_t osm_node_id_t;
typedef boost::unordered_map<std::string, std::string> osm_tags_t;
typedef boost::unordered_map<osm_node_id_t, vertex_t> node_map_t;

struct osm_node_t
{
  double lat;
  double lon;
  osm_node_id_t id;
};

struct osm_way_t
{
  std::vector<osm_node_id_t> nodes;
};

void osm_node_to_graph(osm_node_t const & node, node_map_t & node_map,
                       mutable_graph_t & graph)
{
  vertex_t v = graph.add_vertex(vertex_data_t(node.lon, node.lat));
  node_map.insert(std::make_pair(node.id, v));
}

void osm_way_to_graph(osm_way_t & way, bool one_way, bool reversed,
                      node_map_t const & nodes_map, mutable_graph_t & graph)
{  
  if (reversed)
      std::reverse(way.nodes.begin(), way.nodes.end());
  osm_node_id_t prev_node_id = way.nodes.front();
  vertex_t prev_vertex = nodes_map.find(prev_node_id)->second;

  for (std::vector<osm_node_id_t>::const_iterator it =
      ++way.nodes.begin(); it != way.nodes.end(); ++it)
  {
    vertex_t current_vertex = nodes_map.find(*it)->second;

    double dist = get_distance(prev_vertex.data(), current_vertex.data());
    graph.add_edge(prev_vertex, current_vertex, dist);
    if (!one_way)
    {
      graph.add_edge(current_vertex, prev_vertex, dist);
    }

    prev_node_id = *it;
    prev_vertex = current_vertex;
  }
}

bool is_road(osm_way_t const & way, osm_tags_t const & tags, bool * one_way)
{
  bool is_one_way = false;
  osm_tags_t::const_iterator it = tags.find("area");
  if (it != tags.end())
  {
    if (it->second != "no")
    {
      return false;
    }
  }

  it = tags.find("highway");
  if (it != tags.end())
  {
    std::string const & highway_value = it->second;
    if (highway_value == "motorway" || highway_value == "motorway_link")
    {
      is_one_way = true;
    }

    if (highway_value != "motorway" && highway_value != "motorway_link"
        && highway_value != "trunk" && highway_value != "trunk_link"
        && highway_value != "primary" && highway_value != "primary_link"
        && highway_value != "secondary" && highway_value != "secondary_link"
        && highway_value != "tertiary" && highway_value != "residential"
        && highway_value != "unclassified" && highway_value != "road"
        && highway_value != "living_street" && highway_value != "track"
        && highway_value != "pedestrian")
    {
      return false;
    }
  }
  else
  {
    return false;
  }  

  if (is_one_way)
  {
    it = tags.find("oneway");
    if (it != tags.end())
    {
      if (it->second == "no" || it->second == "false" || it->second == "0")
      {
        is_one_way = false;
      }
    }
  }

  if (one_way)
  {
    *one_way = is_one_way;
  }
  return true;
}

// Derived must declare following methods:
//    start_el_handler(XML_Char const * name, XML_Char const ** attrs)
//    start_el_handler(XML_Char const * name)
template<class Derived>
struct base_expat_parser_t
{
  base_expat_parser_t(std::istream * input, size_t buffer_size) :
    input_(input), buffer_size_(buffer_size)
  {
    parser_ = XML_ParserCreate(NULL);
  }
  virtual ~base_expat_parser_t()
  {
    XML_ParserFree(parser_);
  }

  void parse_input()
  {
    XML_SetElementHandler(parser_, &handle_start_element, &handle_end_element);
    XML_SetUserData(parser_, static_cast<void*> (this));

    boost::scoped_array<char> buffer(new char[buffer_size_]);
    
    while (!input_->eof())
    {
      input_->read(buffer.get(), buffer_size_);
      size_t read = input_->gcount();
      XML_Parse(parser_, buffer.get(), read, false);
    }

    XML_Parse(parser_, NULL, 0, true);
  }
private:
  static void handle_start_element(void * user_data, XML_Char const * name,
                                   XML_Char const ** attrs)
  {
    static_cast<Derived*> (user_data)->start_el_handler(name, attrs);
  }

  static void handle_end_element(void * user_data, XML_Char const * name)
  {
    static_cast<Derived*> (user_data)->end_el_handler(name);
  }

  XML_Parser parser_;
  std::istream * input_;
  size_t buffer_size_;

  // Forbid copy and assignment
  base_expat_parser_t(base_expat_parser_t const &);
  base_expat_parser_t & operator =(base_expat_parser_t const &);
};


struct progress_log_t
{
  progress_log_t()
   : nodes_processed(0), ways_processed(0)
  {
  }
  size_t nodes_processed;
  size_t ways_processed;  
};


std::ostream & operator << (std::ostream & lhs, progress_log_t const & rhs)
{
  lhs << "Processed: " 
      << rhs.nodes_processed << " nodes, " 
      << rhs.ways_processed << " ways.";
}

struct expat_parser_t: base_expat_parser_t<expat_parser_t>
{
  static size_t const DEFAULT_BUFFER_SIZE = 65536; // 64K

  expat_parser_t(mutable_graph_t & graph, std::istream * input, size_t buffer_size = DEFAULT_BUFFER_SIZE) :
    base_expat_parser_t<expat_parser_t>(input, buffer_size),
    graph_(graph),
    unreported_tags_(0)
  {
  }
  
  
  progress_log_t get_progress_log() const
  {
    return progress_log_;
  }
private:
  typedef std::basic_string<XML_Char> xml_string_t;
  void start_el_handler(XML_Char const * name, XML_Char const ** attrs)
  {
    xml_string_t sname(name);
    if (sname == "node")
    {
      want_tags_ = true;
      for (size_t i = 0; attrs[i]; i += 2)
      {
        xml_string_t aname(attrs[i]);
        xml_string_t aval(attrs[i + 1]);
        if (aname == "lat")
        {
          node_.lat = boost::lexical_cast<double>(aval);
        }
        else if (aname == "lon")
        {
          node_.lon = boost::lexical_cast<double>(aval);
        }
        else if (aname == "id")
        {
          node_.id = boost::lexical_cast<osm_node_id_t>(aval);
        }
      }
    }
    else if (sname == "way")
    {
      want_tags_ = true;
      want_nds_ = true;
    }
    else if (sname == "nd" && want_nds_)
    {
      for (size_t i = 0; attrs[i]; i += 2)
      {
        xml_string_t aname(attrs[i]);
        xml_string_t aval(attrs[i + 1]);
        if (aname == "ref")
        {
          way_.nodes.push_back(boost::lexical_cast<osm_node_id_t>(aval));
        }
      }
    }
    else if (sname == "tag" && want_tags_)
    {
      std::string key;
      std::string val;
      for (size_t i = 0; attrs[i]; i += 2)
      {
        xml_string_t aname(attrs[i]);
        xml_string_t aval(attrs[i + 1]);
        if (aname == "k")
        {
          key = aval;
        }
        else if (aname == "v")
        {
          val = aval;
        }
      }

      tags_.insert(std::make_pair(key, val));
    }
  }

  void end_el_handler(XML_Char const * name)
  {
    xml_string_t sname(name);
    if (sname == "node")
    {
      osm_node_to_graph(node_, node_map_, graph_);
      tags_.clear();
      want_tags_ = false;
      ++progress_log_.nodes_processed;
    }
    else if (sname == "way")
    {
      bool one_way;
      if (is_road(way_, tags_, &one_way))
      {
        bool reversed = false;
        if (one_way)          
        {
          osm_tags_t::const_iterator it = tags_.find("oneway");
          if (it != tags_.end())
            reversed = it->second == "-1";
        }
        osm_way_to_graph(way_, one_way, reversed, node_map_, graph_);
      }
      way_.nodes.clear();
      tags_.clear();
      want_tags_ = false;
      want_nds_ = false;
      ++progress_log_.ways_processed;
    }
    
    ++unreported_tags_;
    if (unreported_tags_ >= 500)
    {
      std::cout << "\r" << progress_log_;
      std::cout.flush();
      unreported_tags_ = 0;
    }
  }

  mutable_graph_t & graph_;
  osm_node_t node_;
  osm_way_t way_;
  osm_tags_t tags_;
  node_map_t node_map_;
  bool want_tags_;
  bool want_nds_;
    
  progress_log_t progress_log_;
  size_t unreported_tags_;
 
  friend struct base_expat_parser_t<expat_parser_t>;
};
} // namespace


void convert_to_graph(std::istream & input, mutable_graph_t & out_graph)
{
  expat_parser_t parser(out_graph, &input);
  parser.parse_input();    
  //std::cout << "\r" << parser.get_progress_log() << std::endl;
}
