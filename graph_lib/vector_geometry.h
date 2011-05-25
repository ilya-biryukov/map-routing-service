#pragma once

#include <cmath>

/**
 * @brief Vector on a plane
 */
struct vector2_t
{
  vector2_t(double x, double y);  
  
  double x() const;
  void   set_x(double x);

  double y() const;
  void   set_y(double y);

  double length() const;  

  vector2_t & operator *= (double n);  
  vector2_t & operator /= (double n);
  vector2_t & operator += (vector2_t const & rhs);
  vector2_t & operator -= (vector2_t const & rhs);
  
private:
  double x_;
  double y_;  
};

/**
 * @brief Computes scalar product of two vectors
 * @param v1 First vector
 * @param v2 Second vector
 * @return Scalar product of v1 and v2
 */
double scalar_product(vector2_t const & v1, vector2_t const & v2);

// Implementation
inline vector2_t::vector2_t(double x, double y)
  : x_(x), y_(y)
{     
}

inline double vector2_t::x() const
{
  return x_;
}

inline double vector2_t::y() const
{
  return y_;
}

inline void vector2_t::set_x(double x)
{
  x_ = x;
}

inline void vector2_t::set_y(double y)
{
  y_ = y;    
}  

inline double vector2_t::length() const
{
  return std::sqrt(x_ * x_ + y_ * y_);
}

inline vector2_t & vector2_t::operator *= (double n)
{
  x_ *= n;
  y_ *= n;
  return *this;
}

inline vector2_t & vector2_t::operator /= (double n)
{
  x_ /= n;
  y_ /= n;
  return *this;
}

inline vector2_t & vector2_t::operator += (const vector2_t& rhs)
{
  x_ += rhs.x();
  y_ += rhs.y();
  
  return *this;
}

inline vector2_t & vector2_t::operator -= (const vector2_t& rhs)
{
  x_ -= rhs.x();
  y_ -= rhs.y();
  
  return *this;
}

inline double scalar_product(vector2_t const & v1, vector2_t const & v2)
{
  return v1.x() * v2.x() + v1.y() * v2.y();
}
