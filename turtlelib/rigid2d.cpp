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

    Transform2D::Transform2D(){
        x_diff = 0;
        y_diff = 0;
        rad_diff = 0;
    }

    Transform2D::Transform2D(Vector2D trans){
        x_diff = trans.x;
        y_diff = trans.y;
        rad_diff = 0;
    }

    Transform2D::Transform2D(double radians){
        x_diff = 0;
        y_diff = 0;
        rad_diff = radians;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        x_diff = trans.x;
        y_diff = trans.y;
        rad_diff = radians;
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        // TODO figure this out
        return v;
    }

    // Transform2D Transform2D::inv() const{
    //     // TODO: Implement
    // }
}