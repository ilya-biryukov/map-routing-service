#include <iostream>
#include <string>
#include <istream>
#include <fstream>

#include <boost/program_options.hpp>

#include "mutable_graph.h"
#include "osm_converter.h"

namespace po = boost::program_options;

char const * USAGE_MESSAGE =
    "Usage:\n"
    "osm-to-graph --output-dir=out_dir [OPTIONS]";

namespace
{
void print_usage()
{
  std::cout << USAGE_MESSAGE << std::endl;
}

enum input_mode_en
{
  im_stdin = 0,
  im_file = 1
};

struct program_options
{
  std::string output_directory;
  input_mode_en input_mode;
  std::string input_file;
};

// Returns true if program should continue, false if errors were found and
//   reported to cout
bool parse_args(int argc, char ** argv, program_options & config)
{
  po::options_description generic_desc("Options:");
  generic_desc.add_options()
      ("usage", "Show usage message")
      ("output-dir,o",
          po::value<std::string>(&config.output_directory),
          "Output directory, containing graph data")
      ("input-mode,m",
          po::value<std::string>()->default_value("stdin"),
          "Input mode is 'stdin' or 'file'.")
      ("input-file,i",
          po::value<std::string>(&config.input_file),
          "Input file name(used when input mode is 'file')");

  po::variables_map options;
  po::store(po::parse_command_line(argc, argv, generic_desc), options);
  po::notify(options);

  if (options.count("usage"))
  {
    print_usage();
    return false;
  }

  if (!options.count("output-dir"))
  {
    std::cout << generic_desc << "\n";
    std::cout << "Error: Output-dir must be specified." << std::endl;
    return false;
  }

  std::string input_mode = options["input-mode"].as<std::string>();
  if (input_mode == "file")
  {
    config.input_mode = im_file;
  }
  else if (input_mode == "stdin")
  {
    config.input_mode = im_stdin;
  }
  else
  {
    std::cout << generic_desc << "\n";
    std::cout << "Input mode must be 'file' or 'stdin'" << std::endl;
    return false;
  }

  return true;
}

void osm_to_graph(std::istream & input_stream,
                  std::string const & output_path)
{
  mutable_graph_t graph;  
  std::cout << "Processing osm..." << std::endl;
  convert_to_graph(input_stream, graph);  
  std::cout << "\nDumping to file..." << std::endl;
  graph.dump_to_file(output_path);
  std::cout << "Done." << std::endl;
}
} // namespace

int main(int argc, char ** argv)
{
  program_options config;

  if (!parse_args(argc, argv, config))
  {
    return 1;
  }

  if (config.input_mode == im_stdin)
  {
    osm_to_graph(std::cin, config.output_directory);
  }
  else if (config.input_mode == im_file)
  {
    std::ifstream input(config.input_file.c_str());
    osm_to_graph(input, config.output_directory);
  }
}
