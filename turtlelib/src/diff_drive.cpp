#include <iostream>
#include <cmath> 
#include <string>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive(double wheel_track, double wheel_radius):
        w{WheelState{0.0,0.0}},
        q{Transform2D()},
        track{wheel_track},
        radius{wheel_radius}
    {}

    DiffDrive::DiffDrive(Transform2D config, double wheel_track, double wheel_radius):
        w{WheelState{0.0,0.0}},
        q{config},
        track{wheel_track},
        radius{wheel_radius}
    {}

    Transform2D DiffDrive::get_config(){
        return q;
    }

    double DiffDrive::get_x(){
        return q.translation().x;
    }

    double DiffDrive::get_y(){
        return q.translation().y;
    }

    double DiffDrive::get_phi(){
        return q.rotation();
    }

    WheelState DiffDrive::get_wheels(){
        return w;
    }

    std::ostream & operator<<(std::ostream & os, const WheelState & ws){
        os << "left: " << ws.l << " right: " << ws.r;
        return os;
    }

    void DiffDrive::fk(double new_left, double new_right){
        // taken from MR textbook, pg 524
        double dl = new_left - w.l;
        double dr = new_right - w.r;
        double dphi = (-radius/(2*track))*dl + (radius/(2*track))*dr;
        double dx = 0.5*radius*cos(q.rotation())*dl + 0.5*radius*cos(q.rotation())*dr;
        double dy = 0.5*radius*sin(q.rotation())*dl + 0.5*radius*sin(q.rotation())*dr;
        w.l = new_left;
        w.r = new_right;
        q = Transform2D(Vector2D{q.translation().x + dx,
                                 q.translation().y + dy},
                        q.rotation() + dphi);
    }

    WheelState DiffDrive::ik(Twist2D tw){
        double phi_l = (1./radius)*(tw.linear_velocity().x - track*tw.angular_velocity());
        double phi_r = (1./radius)*(tw.linear_velocity().x + track*tw.angular_velocity());
        return WheelState{phi_l,phi_r};
    }
}