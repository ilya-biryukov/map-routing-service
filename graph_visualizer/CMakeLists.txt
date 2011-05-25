set (graph_visualizer_SRCS      
     vis_widget.cpp
     main.cpp)

set (graph_visualizer_LIBS
     graph)

find_package (Qt4 COMPONENTS QtOpenGL QtCore QtGui REQUIRED)
include (${QT_USE_FILE})
include_directories (${QT_INCLUDE_DIR})
set (graph_visualizer_LIBS ${graph_visualizer_LIBS} ${QT_LIBRARIES})

qt4_wrap_cpp(graph_visualizer_MOC_SRCS 
            vis_widget.h)
set (graph_visualizer_SRCS ${graph_visualizer_SRCS} ${graph_visualizer_MOC_SRCS})

find_package (OpenGL REQUIRED)
include_directories (${OPENGL_INCLUDE_DIR})
set (graph_visualizer_LIBS ${graph_visualizer_LIBS} ${OPENGL_LIBRARIES})

add_executable (graph_visualizer ${graph_visualizer_SRCS})
target_link_libraries (graph_visualizer ${graph_visualizer_LIBS})