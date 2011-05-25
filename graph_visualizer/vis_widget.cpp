#include "vis_widget.h"

#include <QMouseEvent>
#include <QDebug>

#include "vis_graph_adapter.h"
#include "graph_algo.h"

namespace 
{
  QColor const COLOR_VISITED_FORWARD(0xFF, 0x74, 0x00);
  QColor const COLOR_QUEUED_FORWARD(0xFF, 0x96, 0x40);
  QColor const COLOR_VISITED_BACKWARD(0x00, 0x99, 0x99);
  QColor const COLOR_QUEUED_BACKWARD(0x33, 0xCC, 0xCC);
  QColor const COLOR_WAY_VISITED(0x00, 0x00, 0xFF);
  QColor const COLOR_NORMAL_VERTEX(0xA6, 0x00, 0x00);
  QColor const COLOR_SOURCE_VERTEX(0xFE, 0x72, 0x76);
  QColor const COLOR_DESTINATION_VERTEX(0x00, 0xFF, 0x00);
}

VisWidget::VisWidget(QWidget* parent)
  : QGLWidget(parent),
    graph_(NULL),
    zoom_(1.0),
    center_(0.0, 0.0),
    drag_center_(0.0, 0.0),
    drag_point_(0.0, 0.0),
    is_dragging_(false),
    src_id_(-1),
    dst_id_(-1),
    vertex_buffer_(QGLBuffer::VertexBuffer),
    index_buffer_(QGLBuffer::IndexBuffer),
    gl_vertices_(NULL),
    gl_colors_(NULL),
    gl_lines_indicies_(NULL),
    gl_vertices_count_(0),
    gl_lines_count_(0)
{  
  setMouseTracking(true);
}


void VisWidget::setGraph(const graph_t& graph)
{
  releaseGL();
  graph_ = &graph;
}

void VisWidget::runDijkstra()
{  
  if (src_id_ == -1 || dst_id_ == -1)
    return;
  
  clearAll();
    
  vertex_t src = graph_->get_vertex_by_id(src_id_);
  vertex_t dst = graph_->get_vertex_by_id(dst_id_);
    
  std::vector<vertex_t> path_vertices;
  
  details::visual_dijkstra_graph_t adapter(*graph_, this, COLOR_VISITED_FORWARD, COLOR_QUEUED_FORWARD);
  adapter.decrease_key(src, 0.0, src);

  base_dijkstra(adapter, src, dst, std::back_inserter(path_vertices));
    
  for (std::vector<vertex_t>::const_iterator it = path_vertices.begin();
       it != path_vertices.end();
       ++it)
  {
    setPointColor(it->id(), COLOR_WAY_VISITED);
  }
  
  setPointColor(src_id_, COLOR_SOURCE_VERTEX);
  setPointColor(dst_id_, COLOR_DESTINATION_VERTEX);

  updateGL();
}

void VisWidget::runBidiDijkstra()
{
  if (src_id_ == -1 || dst_id_ == -1)
    return;
 
  clearAll();
  
  vertex_t src= graph_->get_vertex_by_id(src_id_);
  vertex_t dst = graph_->get_vertex_by_id(dst_id_);  
  
  std::vector<vertex_t> path_vertices;
  
  details::visual_dijkstra_graph_t forward_adapter(*graph_, this, COLOR_VISITED_FORWARD, COLOR_QUEUED_FORWARD); 
  details::visual_dijkstra_graph_t reversed_adapter(*graph_, this, COLOR_VISITED_BACKWARD, COLOR_QUEUED_BACKWARD);

  forward_adapter.decrease_key(src, 0.0, src);
  reversed_adapter.decrease_key(dst, 0.0, dst);

  
  base_bidirectional_dijkstra(forward_adapter, reversed_adapter, src, dst, std::back_inserter(path_vertices));
    
  for (std::vector<vertex_t>::const_iterator it = path_vertices.begin();
       it != path_vertices.end();
       ++it)
  {
    setPointColor(it->id(), COLOR_WAY_VISITED);
  }
  
  setPointColor(src_id_, COLOR_SOURCE_VERTEX);
  setPointColor(dst_id_, COLOR_DESTINATION_VERTEX);

  updateGL();
}

void VisWidget::runAStar()
{
  if (src_id_ == -1 || dst_id_ == -1)
    return;
  
  clearAll();
    
  vertex_t src = graph_->get_vertex_by_id(src_id_);
  vertex_t dst = graph_->get_vertex_by_id(dst_id_);
    
  std::vector<vertex_t> path_vertices;
  
  details::visual_dijkstra_graph_t adapter(*graph_, this, COLOR_VISITED_FORWARD, COLOR_QUEUED_FORWARD);
  adapter.decrease_key(src, details::potential_dist_f()(src, dst), src);

  base_a_star(adapter, src, dst, std::back_inserter(path_vertices), details::potential_dist_f());
    
  for (std::vector<vertex_t>::const_iterator it = path_vertices.begin();
       it != path_vertices.end();
       ++it)
  {
    setPointColor(it->id(), COLOR_WAY_VISITED);
  }
  
  setPointColor(src_id_, COLOR_SOURCE_VERTEX);
  setPointColor(dst_id_, COLOR_DESTINATION_VERTEX);

  updateGL();
}

void VisWidget::runBidiAStart()
{
  if (src_id_ == -1 || dst_id_ == -1)
    return;
  
  
  clearAll();
  
  vertex_t src= graph_->get_vertex_by_id(src_id_);
  vertex_t dst = graph_->get_vertex_by_id(dst_id_);  
  
  std::vector<vertex_t> path_vertices;
  
  details::visual_dijkstra_graph_t forward_adapter(*graph_, this, COLOR_VISITED_FORWARD, COLOR_QUEUED_FORWARD); 
  details::visual_dijkstra_graph_t reversed_adapter(*graph_, this, COLOR_VISITED_BACKWARD, COLOR_QUEUED_BACKWARD);

  forward_adapter.decrease_key(src, details::bidi_astar_heuristics(src, dst, src, true, details::potential_dist_f()), src);
  reversed_adapter.decrease_key(dst, details::bidi_astar_heuristics(src, dst, dst, false, details::potential_dist_f()), dst);

  
  base_bidirectional_a_star(forward_adapter, reversed_adapter, src, dst, std::back_inserter(path_vertices), details::potential_dist_f());
    
  for (std::vector<vertex_t>::const_iterator it = path_vertices.begin();
       it != path_vertices.end();
       ++it)
  {
    setPointColor(it->id(), COLOR_WAY_VISITED);
  }
  
  setPointColor(src_id_, COLOR_SOURCE_VERTEX);
  setPointColor(dst_id_, COLOR_DESTINATION_VERTEX);

  updateGL();
}



void VisWidget::releaseGL()
{
  vertex_buffer_.unmap();
  index_buffer_.unmap();
  vertex_buffer_.release();
  index_buffer_.release();
  gl_vertices_ = NULL;
  gl_colors_ = NULL;
  gl_lines_indicies_ = NULL;
}

void VisWidget::initializeGL()
{
  size_t const GL_VERTEX_SIZE = 2;
  size_t const GL_VERTEX_BYTES = GL_VERTEX_SIZE * sizeof(GLfloat);
  size_t const GL_COLOR_SIZE = 3 ;
  size_t const GL_COLOR_BYTES = GL_COLOR_SIZE * sizeof(GLfloat);
  
  if (graph_ == NULL)
    return;
  
  gl_vertices_count_ = graph_->get_vertices_count();
  gl_lines_count_ = graph_->get_edges_count() / 2;
  
  vertex_buffer_.create();
  vertex_buffer_.bind();  
  size_t vbuf_bytes_needed = (GL_VERTEX_BYTES + GL_COLOR_BYTES) * gl_vertices_count_;
  vertex_buffer_.allocate(vbuf_bytes_needed);
  
  index_buffer_.create();
  index_buffer_.bind();
  size_t ibuf_bytes_needed = 2 * sizeof(GLuint) * gl_lines_count_;
  index_buffer_.allocate(ibuf_bytes_needed);
  
  gl_vertices_ = static_cast<GLfloat *>(vertex_buffer_.map(QGLBuffer::ReadWrite));
  gl_colors_ = &gl_vertices_[GL_VERTEX_SIZE * gl_vertices_count_];
  
  gl_lines_indicies_ = static_cast<GLuint *>(index_buffer_.map(QGLBuffer::ReadWrite));
  
  qglClearColor(Qt::white);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glPointSize(4.0);
  
  initGLData();
}

void VisWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT);
 
  glVertexPointer(2, GL_FLOAT, 0, 0);
  glColorPointer(3, GL_FLOAT, 0, reinterpret_cast<const GLvoid *>(gl_vertices_count_ * 2 * sizeof(GLfloat)));

  glDrawArrays(GL_POINTS, 0, gl_vertices_count_);
  glDrawElements(GL_LINES, 2 * gl_lines_count_, GL_UNSIGNED_INT, 0);  
}


void VisWidget::resizeGL(int w, int h)
{
  updateGLProjection(center_, 360. / zoom_, 180. / zoom_);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glViewport(0, 0, GLint(w), GLint(h));
}

void VisWidget::updateGLProjection(QPointF center_, float lonDiff, float latDiff)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(center_.x() - lonDiff / 2., 
             center_.x() + lonDiff / 2.,
             center_.y() - latDiff / 2.,
             center_.y() + latDiff / 2.);
}

size_t VisWidget::findClosestVertexId(QPointF coords)
{
  vertex_data_t coords_data(coords.x(), coords.y());
  size_t best_id = -1;  
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < gl_vertices_count_; ++i)
  {
    vertex_data_t v_data(gl_vertices_[2 * i], gl_vertices_[2 * i + 1]);
    double dist = get_distance(coords_data, v_data);
    if (dist < best_dist)
    {
      best_dist = dist;
      best_id = i;
    }
  }
  
  return best_id;
}



QPointF VisWidget::pointToLatLon(QPointF const & pos) const
{
  return QPointF(getLeftLon() + (getRightLon() - getLeftLon()) / width() * pos.x(),
                 getBottomLat() + (getTopLat() - getBottomLat()) / height() * (height() - pos.y()));
}


void VisWidget::mouseMoveEvent(QMouseEvent * ev)
{
  if (is_dragging_)
  {     
    updateGLProjection(drag_center_ - (pointToLatLon(ev->posF()) - drag_point_),
                       360. / zoom_,
                       180. / zoom_);
    updateGL();
  }  
}

void VisWidget::mousePressEvent(QMouseEvent * ev)
{
  if (ev->button() == Qt::LeftButton)
  {    
    drag_center_ = center_;
    drag_point_ = pointToLatLon(ev->posF());
    is_dragging_ = true;    
  }
  
  ev->accept();
}

void VisWidget::mouseReleaseEvent(QMouseEvent * ev)
{
  if (ev->button() == Qt::LeftButton)
  {
    is_dragging_ = false;
    center_ = drag_center_ - (pointToLatLon(ev->posF()) - drag_point_);
    updateGLProjection(center_, 360. / zoom_, 180. / zoom_);
    updateGL();
  }
  
  if (ev->button() == Qt::LeftButton && (ev->modifiers() == Qt::ShiftModifier))
  {
    setSourceVertex(findClosestVertexId(pointToLatLon(ev->posF())));
    updateGL();
  }
  
  if (ev->button() == Qt::LeftButton && (ev->modifiers() == Qt::ControlModifier))
  {
    setDestinationVertex(findClosestVertexId(pointToLatLon(ev->posF())));
    updateGL();
  }
  
  if (ev->button() == Qt::RightButton)
  {
    clearAll();
    updateGL();
    
    if (ev->modifiers() == Qt::ControlModifier)
      runDijkstra();
    else if (ev->modifiers() == Qt::ShiftModifier)
      runBidiDijkstra();
    else if (ev->modifiers() == Qt::AltModifier)
      runAStar();
    
  }

  ev->accept();
}

void VisWidget::wheelEvent(QWheelEvent * ev)
{
  if (ev->delta() > 0)
  {
    zoom_ *= 2.;
  }
  else if (ev->delta() < 0)
  {
    zoom_ /= 2.;
  }
  
  if (zoom_ < 1.)
  {
    zoom_ = 1.;
  }
  
  updateGLProjection(center_, 360. / zoom_, 180. / zoom_);
  updateGL();
  ev->accept();
}


void VisWidget::initGLData()
{
  size_t vertices_count = gl_vertices_count_;
  size_t next_edge = 0;
  for (size_t i = 0; i < vertices_count; ++i)
  {
    vertex_t v = graph_->get_vertex_by_id(i);
    setPointCoordinates(i, v.data().lon(), v.data().lat());
    setPointColor(i, COLOR_NORMAL_VERTEX);
    
    graph_t::edges_range inc_normal = graph_->get_incident_edges(v, EDGE_NORMAL);
    for (graph_t::edges_range::const_iterator it = inc_normal.begin(); it != inc_normal.end(); ++it)
    {
      setEdge(next_edge++, it->source(), it->destination());
    }     
  }
}
void VisWidget::setSourceVertex(size_t id)
{
  if (src_id_ != -1)
    setPointColor(src_id_, COLOR_NORMAL_VERTEX);
  src_id_ = id;
  if (id != -1)
    setPointColor(id, COLOR_SOURCE_VERTEX);  
}

void VisWidget::clearAll()
{
  for (size_t i = 0; i < gl_vertices_count_; ++i)
  {
    if (i != src_id_ && i != dst_id_)
      setPointColor(i, COLOR_NORMAL_VERTEX);
  }  
}


void VisWidget::setDestinationVertex(size_t id)
{
  if (dst_id_ != -1)
    setPointColor(dst_id_, COLOR_NORMAL_VERTEX);
  dst_id_ = id;
  if (id != -1)
    setPointColor(id, COLOR_DESTINATION_VERTEX);
}
