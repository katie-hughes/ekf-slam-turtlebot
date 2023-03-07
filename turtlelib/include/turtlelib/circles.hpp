#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include<iosfwd>

namespace turtlelib
{
  /// @brief Way of grouping the lidar measurements
  struct RangeBearing
  {
    /// @brief Range in m
    double range = 0.0;
    /// @brief Bearing in radians
    double bearing = 0.0;
  };

  /// \brief output a range-bearing as "Range: x Bearing: y"
    /// os - stream to output to
    /// rb - the RangeBearing to print
    std::ostream & operator<<(std::ostream & os, const RangeBearing & rb);
}