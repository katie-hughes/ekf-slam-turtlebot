#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include <iosfwd>
#include <vector>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
  /// @brief Specification of polar coordinates (what lidar uses)
  struct Polar
  {
    /// @brief Range in m
    double r = 0.0;
    /// @brief Bearing in radians
    double theta = 0.0;
  };

  /// @brief Circle representation in the 2D plane
  struct Circle
  {
    /// @brief X coordinate of origin (m)
    double x = 0.0;
    /// @brief Y coordinate of origin (m)
    double y = 0.0;
    /// @brief Radius of circle (m)
    double r = 0.0;
  };

  /// \brief output a Polar coordinate as "Theta: x r: y"
  /// os - stream to output to
  /// p - the coordinates to print
  std::ostream & operator<<(std::ostream & os, const Polar & p);

  /// \brief output a Circle as Center: (x,y) Radius: R"
  /// os - stream to output to
  /// c - the coordinates to print
  std::ostream & operator<<(std::ostream & os, const Circle & c);

  /// @brief Obtain the euclidean distance between two Polar coordinates
  /// @param p1 polar coordinate 1
  /// @param p2 polar coordinate 2
  /// @return distance between the two points
  double polarDistance(const Polar p1, const Polar p2);


  /// @brief simple check if point is at (0,0)
  /// @param p point
  /// @return true if the point is at the origin, false otherwise
  bool atOrigin(const Polar p);

  /// @brief Convert polar coordinate to euclidean 2D coordinate
  /// @param p Polar coordinate
  /// @return coordinate in (x,y)
  Vector2D toVector(const Polar p);

  /// @brief Detect a 2D circle from a list of points
  /// @param cluster the input points. Must have a length of at least 4
  /// @return Circle that best fits these points
  Circle detectCircle(const std::vector<Vector2D> cluster);

  /// @brief Detect whether a series of points should be classified as a circle
  /// @param cluster input points in the 2D plane. Must have a length of at least 1
  /// @param std_threshold Maximum standard devation in angle to be considered circle. Default = 0.15
  /// @param mean_lo_threshold Minimum angle mean to be considered a circle. Default = 90 degrees
  /// @param mean_hi_threshold Maximum angle mean to be considered a circle. Default = 130 degrees
  /// @return boolean signifying if cluster should be classified as circle
  bool isCircle(const std::vector<Vector2D> cluster, double std_threshold=0.15,
                                                     double mean_lo_threshold=1.50,
                                                     double mean_hi_threshold=2.50);
}