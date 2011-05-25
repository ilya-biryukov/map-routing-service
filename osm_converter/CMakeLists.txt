set (osm_converter_SRCS 
     main.cpp
     osm_converter.cpp)
set (osm_converter_LIBS 
     graph)


find_package (Boost COMPONENTS program_options REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
set (osm_converter_LIBS ${osm_converter_LIBS} ${Boost_LIBRARIES})

find_package (EXPAT)
include_directories (${EXPAT_INCLUDE_DIRS})
set (osm_converter_LIBS ${osm_converter_LIBS} ${EXPAT_LIBRARIES})

add_executable(osm_converter ${osm_converter_SRCS})
target_link_libraries(osm_converter ${osm_converter_LIBS})
