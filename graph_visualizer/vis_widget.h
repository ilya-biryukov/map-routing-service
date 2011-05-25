#pragma once

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>

#include "graph.h"

class VisWidget : public QGLWidget
{
  Q_OBJECT
public:
  VisWidget(QWidget * parent = 0);
  
  void setGraph(graph_t const & graph);  
  
  void setPointColor(size_t id, QColor color);
  void clearAll();
  
public slots:
  void runDijkstra();
  void runBidiDijkstra();
  void runAStar();
  void runBidiAStart();
protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);  
  
  virtual void mouseMoveEvent(QMouseEvent * ev);
  virtual void mousePressEvent(QMouseEvent * ev);    
  virtual void mouseReleaseEvent(QMouseEvent * ev);
 
  virtual void wheelEvent(QWheelEvent * ev);
private:
  void releaseGL();
  void setPointCoordinates(size_t id, float lon, float lat);
  void setEdge(size_t id, size_t v1, size_t v2);  
  void initGLData();
  void updateGLProjection(QPointF center_, float lonDiff, float latDiff);
  size_t findClosestVertexId(QPointF coords);
  
  float getLeftLon() const;
  float getRightLon() const;
  float getBottomLat() const;
  float getTopLat() const;  
  // From window coordinates to lat lon
  QPointF pointToLatLon(QPointF const & pos) const;
  
  void setSourceVertex(size_t id);
  void setDestinationVertex(size_t id);

  
  
  graph_t const * graph_;  
  /* Zoom */
  float zoom_;  
  /* Dragging */
  QPointF center_;
  QPointF drag_center_;
  QPointF drag_point_;
  bool is_dragging_;
  /* Vertex coloring and paths */
  size_t src_id_;
  size_t dst_id_;    
  /* OpenGL stuff */
  QGLBuffer vertex_buffer_;
  QGLBuffer index_buffer_;  
  GLfloat * gl_vertices_;
  GLfloat * gl_colors_;
  GLuint  * gl_lines_indicies_;    
  size_t gl_vertices_count_;
  size_t gl_lines_count_;
};

inline void VisWidget::setPointCoordinates(size_t id, float lon, float lat)
{
  size_t i = 2 * id;
  gl_vertices_[i] = lon;
  gl_vertices_[i + 1] = lat;
}


inline void VisWidget::setPointColor(size_t id, QColor color)
{
  if (id >= gl_vertices_count_)
    return;

  size_t i = 3 * id;
  
  gl_colors_[i] = color.redF();
  gl_colors_[i + 1] = color.greenF();
  gl_colors_[i + 2] = color.blueF();
}

inline void VisWidget::setEdge(size_t id, size_t v1, size_t v2)
{
  size_t i = 2 * id;  
  gl_lines_indicies_[i] = v1;
  gl_lines_indicies_[i + 1] = v2;
}

inline float VisWidget::getLeftLon() const
{
  return center_.x() - 180. / zoom_;
}

inline float VisWidget::getRightLon() const
{
  return center_.x() + 180. / zoom_;  
}

inline float VisWidget::getBottomLat() const
{
  return center_.y() - 90. / zoom_;
}

inline float VisWidget::getTopLat() const
{
  return center_.y() + 90. / zoom_;
}