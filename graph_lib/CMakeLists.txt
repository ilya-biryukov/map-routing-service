set (graph_lib_SRCS 
     graph.cpp
     data_accessor.cpp
     graph_algo_adapters.cpp    
     spatial_index.cpp) 

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

find_package (Boost COMPONENTS filesystem system REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
set (graph_lib_LIBS ${Boost_LIBRARIES})

add_library(graph ${graph_lib_SRCS})
target_link_libraries(graph ${graph_lib_LIBS})