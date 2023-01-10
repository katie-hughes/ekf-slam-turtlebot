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
        char c = is.peek();
        if (c == '['){
            // grab the bracket
            c = is.get();
            // the next two are the chars
            is >> v.x >> v.y;
            // v.x = is.get();
            // v.y = is.get();
            // grab final bracket
            c = is.get();
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }
}