cmake_minimum_required (VERSION 2.6)

set (shortest_paths_SRCS
     "main.cpp")
set (shortest_paths_LIBS 
     graph)

find_package (Boost COMPONENTS thread REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
set (shortest_paths_LIBS ${shortest_paths_LIBS} ${Boost_LIBRARIES})

find_package (FCGIPP REQUIRED)
include_directories (${FCGIPP_INCLUDE_DIRS})
set (shortest_paths_LIBS ${shortest_paths_LIBS} ${FCGIPP_LIBRARIES})


find_package (JSON REQUIRED)
include_directories (${JSON_INCLUDE_DIRS})
set (shortest_paths_LIBS ${shortest_paths_LIBS} ${JSON_LIBRARIES})

add_definitions(-Wall)

add_executable (shortest_paths ${shortest_paths_SRCS})
target_link_libraries (shortest_paths ${shortest_paths_LIBS})
