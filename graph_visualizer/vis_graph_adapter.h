#pragma once

#include <functional>

#include "graph_data.h"
#include "graph_algo_adapters.h"
#include "graph.h"

#include "vis_widget.h"

namespace details
{
struct visual_dijkstra_graph_t
{
  visual_dijkstra_graph_t(graph_t const & graph, VisWidget * widget, QColor visited_color, QColor queued_color)
      : base_adapter_(graph), widget_(widget), visited_color_(visited_color), queued_color_(queued_color)
  {
  }

  std::pair<double, vertex_t> next() const
  {
    return base_adapter_.next();
  }
  
  void dequeue()
  { 
    widget_->setPointColor(base_adapter_.next().second.id(), visited_color_);
    base_adapter_.dequeue();
  }

  void decrease_key(vertex_t v, double new_dist, vertex_t new_pred)
  {
    if (get_dist(v) == std::numeric_limits<double>::max())
    {
      widget_->setPointColor(v.id(), queued_color_);
    }
    base_adapter_.decrease_key(v, new_dist, new_pred);
  }

  bool empty() const
  {
    return base_adapter_.empty();
  }

  vertex_t get_predecessor(vertex_t v) const
  {
    return base_adapter_.get_predecessor(v);
  }

  double get_dist(vertex_t v) const
  {
    return base_adapter_.get_dist(v);
  }

  bool was_processed(vertex_t v) const
  {
    return base_adapter_.was_processed(v);
  }
  
  graph_t const & get_graph() const
  {
    return base_adapter_.get_graph();
  }
private:
  details::dijkstra_graph_t base_adapter_;
  
  VisWidget * widget_;
  QColor visited_color_;
  QColor queued_color_;
};
} // namespace details
