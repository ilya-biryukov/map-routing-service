#include "graph.h"

#include <string>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/shared_ptr.hpp>

namespace interproc = boost::interprocess;

namespace
{
boost::shared_ptr<interproc::mapped_region> map_to_mem(std::string const & filename,
                                                       interproc::mode_t mode,
                                                       size_t filesize)
{
  interproc::file_mapping f_mapping(filename.c_str(), mode);  
  return boost::shared_ptr<interproc::mapped_region>(
      new interproc::mapped_region(f_mapping, mode, 0, filesize));;
}
} // namespace

graph_t::graph_t(std::string const & path_to_graph)
{
  size_t data_size = boost::filesystem::file_size(path_to_graph);
  init(path_to_graph, data_size);
}

void graph_t::init(const std::string & path_to_graph, size_t graph_data_size)
{
  mapped_data_ptr_ = map_to_mem(path_to_graph, interproc::read_only, graph_data_size);
  data_accessor_ptr_.reset(new details::data_accessor_t(mapped_data_ptr_->get_address()));
}
