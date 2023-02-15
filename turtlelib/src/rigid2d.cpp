#include <iostream>
// Is including math allowed?
#include <cmath> 
#include <string>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    // help: https://www.learncpp.com/cpp-tutorial/printing-inherited-classes-using-operator/
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        // Remove whitespace
        is >> std::ws;
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
        // Read \n character
        c = is.get();
        return is;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & scalar){
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
        return lhs += rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
        return lhs -= rhs;
    }

    Vector2D operator*(Vector2D vec, const double & scalar){
        return vec*=scalar;
    }

    double dot(Vector2D v1, Vector2D v2){
        return v1.x*v2.x + v1.y*v2.y;
    }

    double magnitude(Vector2D vec){
        return sqrt(vec.x*vec.x + vec.y*vec.y);
    }

    double angle(Vector2D v1, Vector2D v2){
        return atan2(v1.y, v1.x) - atan2(v2.y, v2.x);
    }

    double normalize_angle(double rad){
        // this method has float issues if loop iterates more than just a few times...
        while (rad > PI){
            rad -= 2*PI;
        }
        while (rad <= -1*PI){
            rad += 2*PI;
        }
        return rad;
    }

    Vector2D normalize(Vector2D v){
        const auto norm = sqrt(v.x*v.x + v.y*v.y);
        v.x = v.x/norm;
        v.y = v.y/norm;
        return v;
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
        const auto new_x = linear.x + v.x*cos(angular) - v.y*sin(angular);
        const auto new_y = linear.y + v.x*sin(angular) + v.y*cos(angular);
        return Vector2D{new_x, new_y};
    }

    Transform2D Transform2D::inv() const{
        const auto new_angular = -angular;
        const auto new_x = -linear.x*cos(angular) - linear.y*sin(angular);
        const auto new_y =  linear.x*sin(angular) - linear.y*cos(angular);
        return Transform2D{Vector2D{new_x, new_y}, new_angular};
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        const auto new_angular = angular + rhs.angular;
        const auto new_x = rhs.linear.x*cos(angular) - rhs.linear.y*sin(angular) + linear.x;
        const auto new_y = rhs.linear.x*sin(angular) + rhs.linear.y*cos(angular) + linear.y;

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
        // Remove whitespace
        is >> std::ws;
        double input_x, input_y, input_ang;
        char c = is.peek();
        if (c=='d'){
            std::string label_ang, label_x, label_y;
            is >> label_ang >> input_ang >> label_x >> input_x >> label_y >> input_y;
        } else{
            is >> input_ang >> input_x >> input_y;
        }
        // Read \n character
        c = is.get();
        // put values into Transform2D Object
        tf = Transform2D(Vector2D{input_x, input_y},deg2rad(input_ang));
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        // Is it actually this easy?
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
        // Remove whitespace
        is >> std::ws;
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
        // Read \n character
        c = is.get();
        // put these inputs into the tw vector?
        tw = Twist2D(input_w, Vector2D{input_vx, input_vy});
        return is;
    }

    Twist2D Transform2D::operator()(Twist2D tw) const{
        // To do something like Va = Tab(Vb)
        const auto wz = tw.angular_velocity();
        const auto v = tw.linear_velocity();
        const auto new_vx =  linear.y*wz + v.x*cos(angular) - v.y*sin(angular);
        const auto new_vy = -linear.x*wz + v.x*sin(angular) - v.y*cos(angular);
        return Twist2D{wz, Vector2D{new_vx,new_vy}};
    }

    Transform2D integrate_twist(Twist2D tw){
        // two cases, if it has angular component or no.
        if (tw.angular_velocity() == 0){
            // tw.linear_velocity() gives a Vector2D
            // components are x in m/s and y in m/s
            // but this is over unit time, so m/s ~ m.
            return Transform2D(tw.linear_velocity());
        } else {
            // Twist has angular component.
            const auto xs = tw.linear_velocity().y/tw.angular_velocity();
            const auto ys = -1*tw.linear_velocity().x/tw.angular_velocity();
            const auto Tsb = Transform2D(Vector2D{xs,ys});
            const auto Tssprime = Transform2D(tw.angular_velocity());
            const auto Tsbprime = Tssprime*Tsb;
            const auto Tbbprime = (Tsb.inv())*Tsbprime;
            return Tbbprime;
        }
    }

    double distance(double x1, double y1, double x2, double y2){
        return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
    }
}