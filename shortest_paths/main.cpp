#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <boost/noncopyable.hpp>
#include <fastcgi++/request.hpp>
#include <fastcgi++/manager.hpp>
#include <libjson/libjson.h>

#include "graph.h"
#include "graph_data.h"
#include "graph_algo.h"
#include "spatial_index.h"
#include "graph_services.h"
#include "json_writers.h"

namespace
{
struct shared_data
{
  static graph_t const & get_graph()
  {
    return *graph_ptr_;
  }

  static spatial_index_t const & get_spatial_index()
  {
    return *spatial_index_ptr_;
  }

  static void set_graph(graph_t * graph_ptr)
  {
    graph_ptr_ = graph_ptr;
  }

  static void set_spatial_index(spatial_index_t * s_index_ptr)
  {
    spatial_index_ptr_ = s_index_ptr;
  }

private:
  static graph_t *  graph_ptr_;
  static spatial_index_t * spatial_index_ptr_;
};

graph_t * shared_data::graph_ptr_ = 0;
spatial_index_t * shared_data::spatial_index_ptr_ = 0;
} // namespace

struct ShortestPathRequest: Fastcgipp::Request<char>
{
  ShortestPathRequest()
    : services_(&shared_data::get_graph(), &shared_data::get_spatial_index())
  {    
  }
  
  JSONNode runShortestPaths(wgs82_point_t const & src, wgs82_point_t const & dst)
  {
    JSONNode path(JSON_ARRAY);
    
    json_output_iter_t out_iter(&path, &shared_data::get_graph());
    services_.find_shortest_path(src, dst, out_iter);
    
    return path;
  }
  
  JSONNode showNearestEdges(wgs82_point_t const & point)
  {
    JSONNode edges(JSON_ARRAY);
    
    json_output_iter_t out_iter(&edges, &shared_data::get_graph());
    services_.mark_nearest_edges(point, out_iter);
    
    return edges;
  }
  
  bool response()
  {
    Gets const & gets = environment().gets;
    vertex_data_t pt_src(from_gets<float> (gets, "lng1"),
                         from_gets<float> (gets, "lat1"));
    vertex_data_t pt_dst(from_gets<float> (gets, "lng2"),
                         from_gets<float> (gets, "lat2"));
    out << "Content-Type: application/json;\r\n\r\n";
    
    JSONNode output(JSON_NODE);
    JSONNode path = runShortestPaths(pt_src, pt_dst);
    path.set_name("path");
    output.push_back(path);
    
    JSONNode edges_src = showNearestEdges(pt_src);
    edges_src.set_name("edges_source");
    output.push_back(edges_src);
    
    JSONNode edges_dst = showNearestEdges(pt_dst);
    edges_dst.set_name("edges_destination");
    output.push_back(edges_dst);
    
    out << output.write();
    
    return true;
  }
private:
  typedef Fastcgipp::Http::Environment<char>::Gets Gets;
  
  graph_services_t services_;

  template<class T>
  static T from_gets(Gets const & gets, std::string const & name,
                     T const & def = T())
  {
    Gets::const_iterator it = gets.find(name);
    if (it == gets.end())
    {
      return def;
    }
    else
    {
      return boost::lexical_cast<T>(it->second);
    }
  }
};

int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    std::cout << "Usage: shortest-paths filename" << std::endl;
    return -1;
  }
  
  graph_t graph(argv[1]);
  spatial_index_t spatial_index(graph, 30000, 30000);
  shared_data::set_graph(&graph);
  shared_data::set_spatial_index(&spatial_index);
  
  Fastcgipp::Manager<ShortestPathRequest> manager;  
  manager.handler();
}
