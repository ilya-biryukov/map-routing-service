#pragma once

double const PI =3.14159265358979323846;
double const EARTH_RADIUS = 6371000.;

inline double deg_to_rad(double deg)
{
  return deg * PI / 180.;
}

inline double rad_to_deg(double rad)
{
  return rad * 180.0 / PI;
}
