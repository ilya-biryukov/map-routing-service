cmake_minimum_required (VERSION 2.6)

project (shortest-paths)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

add_subdirectory (graph_lib)
include_directories (graph_lib)
add_subdirectory (osm_converter)
add_subdirectory (shortest_paths)
