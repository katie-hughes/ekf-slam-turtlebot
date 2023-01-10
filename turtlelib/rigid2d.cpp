#include <iostream>
#include "rigid2d.hpp"

namespace turtlelib
{
    // help: https://www.learncpp.com/cpp-tutorial/printing-inherited-classes-using-operator/
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << " [" << v.x << " " << v.y << "] ";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        return is;
    }
}



