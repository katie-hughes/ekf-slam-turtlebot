#include "turtlelib/circles.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const RangeBearing & rb){
        os << "Bearing: " << rb.bearing << "\tRange: " << rb.range;
        return os;
    }
}