#include <iostream>
// Is including math allowed?
#include <math.h> 
#include "rigid2d.hpp"

namespace turtlelib
{
    // help: https://www.learncpp.com/cpp-tutorial/printing-inherited-classes-using-operator/
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        char c = is.peek();
        if (c == '['){
            // grab the bracket
            c = is.get();
            // the next two are the chars
            is >> v.x >> v.y;
            // grab final bracket
            c = is.get();
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }

    Transform2D::Transform2D():
        linear{Vector2D{0,0}},
        angular{0}
        {}

    Transform2D::Transform2D(Vector2D trans):
        linear{trans},
        angular{0}
        {}

    Transform2D::Transform2D(double radians):
        linear{Vector2D{0,0}},
        angular{radians}
        {}

    Transform2D::Transform2D(Vector2D trans, double radians):
        linear{trans},
        angular{radians}
        {}

    Vector2D Transform2D::operator()(Vector2D v) const{
        // To do something like va = Tab(vb)
        double new_x = linear.x + v.x*cos(angular) - v.y*sin(angular);
        double new_y = linear.y + v.x*sin(angular) + v.y*cos(angular);
        return Vector2D{new_x, new_y};
    }

    Transform2D Transform2D::inv() const{
        double new_angular = -angular;
        double new_x = -linear.x*cos(angular) - linear.y*sin(angular);
        double new_y =  linear.x*sin(angular) - linear.y*cos(angular);
        return Transform2D{Vector2D{new_x, new_y}, new_angular};
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        double new_angular = angular + rhs.angular;
        double new_x = rhs.linear.x*cos(angular) - rhs.linear.y*sin(angular) + linear.x;
        double new_y = rhs.linear.x*sin(angular) + rhs.linear.y*cos(angular) + linear.y;

        angular = new_angular;
        linear.x = new_x;
        linear.y = new_y;
        return *this;
    }

    Vector2D Transform2D::translation() const{
        return linear;
    };

    double Transform2D::rotation() const{
        return angular;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.angular) << " x: " << tf.linear.x << " y: " << tf.linear.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        // it is unhappy with this. Why?
        // Below would be fine if this was friend function but it is not
        // is >> tf.angular >> tf.linear.x >> tf.linear.y;
        double input_x, input_y, input_ang;
        char c = is.peek();
        if (c=='d'){
            // TODO dont' use these use std::string package
            char label_ang[10], label_x[10], label_y[10];
            is >> label_ang >> input_ang >> label_x >> input_x >> label_y >> input_y;
        } else{
            is >> input_ang >> input_x >> input_y;
        }
        // put these inputs into the tf vector?
        tf = Transform2D(Vector2D{input_x, input_y},deg2rad(input_ang));
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        // TODO: implement. Is it this easy?
        return lhs*=rhs;
    }

    Twist2D::Twist2D():
        angular{0},
        linear{Vector2D{0,0}}
        {}

    Twist2D::Twist2D(double w):
        angular{w},
        linear{Vector2D{0,0}}
        {}
    
    Twist2D::Twist2D(Vector2D v):
        angular{0},
        linear{v}
        {}

    Twist2D::Twist2D(double w, Vector2D v):
        angular{w},
        linear{v}
        {}

    double Twist2D::angular_velocity() const{
        return angular;
    };

    Vector2D Twist2D::linear_velocity() const{
        return linear;
    };

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.angular << " " << tw.linear.x << " " << tw.linear.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        double input_w, input_vx, input_vy;
        char c = is.peek();
        if (c=='['){
            // grab the bracket
            c = is.get();
            // the next two are the chars
            is >> input_w >> input_vx >> input_vy;
            // grab final bracket
            c = is.get();
        } else{
            is >> input_w >> input_vx >> input_vy;
        }
        // put these inputs into the tw vector?
        tw = Twist2D(input_w, Vector2D{input_vx, input_vy});
        return is;
    }
}