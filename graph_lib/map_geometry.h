#pragma once

#include <cmath>
#include <boost/iterator/iterator_concepts.hpp>

#include "math_utils.h"


enum coords_t
{  
  COORDS_MERCATOR,
  COORDS_WGS82,
  COORDS_GRID
};

/**
 * @brief A point on earth in specific coordinate systems.
 */
template<coords_t Coords>
struct point2_t
{
  point2_t(double x, double y);
  template<coords_t Coords2>
  explicit point2_t(point2_t<Coords2> const & source);
  
  double x() const;
  double y() const;
  
  void set_x(double x);
  void set_y(double y);
private:
  double x_;
  double y_;
};

template<coords_t Coords>
bool operator == (point2_t<Coords> const & lhs, point2_t<Coords> const & rhs);


/**
 * @brief A point on earth in WGS-82 coordinates.
 */
struct wgs82_point_t : public point2_t<COORDS_WGS82>
{
  wgs82_point_t(double lon, double lat);
  template<coords_t Coords2>
  explicit wgs82_point_t(point2_t<Coords2> const & source);
  
  double lat() const;
  void   set_lat(double lat);
  
  double lon() const;  
  void   set_lon(double lon);
};


/**
 * @brief A point on earth in Mercator projection coordinates
 */
typedef point2_t<COORDS_MERCATOR> mercator_point_t;

/**
 * @brief A point on earth in grid coordinates.(spatial_index_t specific)
 */
typedef point2_t<COORDS_GRID> grid_point_t;


/**
 * @brief Evaluates distance between two points.
 * 
 * @param pt1 First point
 * @param pt2 Second point
 * 
 * @return Distance between points in meters
 */
template<coords_t Coords>
double get_distance(point2_t<Coords> const & pt1, point2_t<Coords> const & pt2);

// Implementation
namespace details
{
/**
 * @brief Functor used for tranfroming between coordinate systems.
 */
template<coords_t SourceCoords, coords_t TargetCoords>
struct coords_transformer_f
{
  point2_t<TargetCoords> operator () (point2_t<SourceCoords> const & point);
};

/**
 * @brief Partial specialization when source and target coordinate systems are the same.
 */
template<coords_t Coords>
struct coords_transformer_f<Coords, Coords>
{
  point2_t<Coords> operator () (point2_t<Coords> const & point)
  {
    return point;
  }
};

/**
 * @brief Transformer from WGS82 to Mercator
 */
template<>
struct coords_transformer_f<COORDS_MERCATOR, COORDS_WGS82>
{
  point2_t<COORDS_MERCATOR> operator () (point2_t<COORDS_WGS82> const & point)
  {
    return point2_t<COORDS_MERCATOR>(deg_to_rad(point.x()), 
                                     std::log(std::tan(PI / 4.0 + deg_to_rad(point.y()) / 2.0)));
  }
};

/**
 * @brief Transformer from Mercator to WGS82
 */
template<>
struct coords_transformer_f<COORDS_WGS82, COORDS_MERCATOR>
{
  point2_t<COORDS_WGS82> operator () (point2_t<COORDS_MERCATOR> const & point)
  {
    return point2_t<COORDS_WGS82>(rad_to_deg(point.x()), 
                                  rad_to_deg(2.0 * std::atan(std::exp(point.y())) - PI / 2.0));
  }
};
} // namespace details;

template<coords_t Coords>
point2_t<Coords>::point2_t(double x, double y)
  : x_(x), y_(y)
{
}

template<coords_t Coords>
template<coords_t Coords2>
point2_t<Coords>::point2_t(point2_t<Coords2> const & source)
{  
  *this = point2_t(details::coords_transformer_f<Coords, Coords2>()(source));
}

template<coords_t Coords>
double point2_t<Coords>::x() const
{
  return x_;
}

template<coords_t Coords>
double point2_t<Coords>::y() const
{
  return y_;  
}

template<coords_t Coords>
void point2_t<Coords>::set_x(double x)
{
  x_ = x;  
}

template<coords_t Coords>
void point2_t<Coords>::set_y(double y)
{
  y_ = y;
}

inline double wgs82_point_t::lat() const
{
  return y();
}

inline double wgs82_point_t::lon() const
{
  return x();
}

inline wgs82_point_t::wgs82_point_t(double lon, double lat)
  : point2_t<COORDS_WGS82>(lon, lat)
{
}

template<coords_t Coords2>
wgs82_point_t::wgs82_point_t(point2_t<Coords2> const & source)
  : point2_t<COORDS_WGS82>(source)
{  
}

inline void wgs82_point_t::set_lon(double lon)
{
  set_x(lon);
}

inline void wgs82_point_t::set_lat(double lat)
{
  set_y(lat);
}

template<>
inline double get_distance<COORDS_WGS82>(point2_t<COORDS_WGS82> const & pt1, point2_t<COORDS_WGS82> const & pt2)
{
  double lat1 = deg_to_rad(pt1.y());
  double lon1 = deg_to_rad(pt1.x());  
  double lat2 = deg_to_rad(pt2.y());
  double lon2 = deg_to_rad(pt2.x());
  
  double cos_lat1 = std::cos(lat1);
  double cos_lat2 = std::cos(lat2);
  double sin_lat1 = std::sin(lat1);
  double sin_lat2 = std::sin(lat2);

  double dx = cos_lat2 * std::cos(lon2) - cos_lat1 * std::cos(lon1);
  double dy = cos_lat2 * std::sin(lon2) - cos_lat1 * std::sin(lon1);
  double dz = sin_lat2 - sin_lat1;
  
  double ch = std::sqrt(dx * dx + dy * dy + dz * dz);
    
  return EARTH_RADIUS * 2. * std::asin(ch / 2.); 
}

template<coords_t Coords>
bool operator == (point2_t<Coords> const & lhs, point2_t<Coords> const & rhs)
{
  return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}
