#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
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

    Transform2D DiffDrive::get_config() const{
        return q;
    }

    double DiffDrive::get_x() const{
        return q.translation().x;
    }

    double DiffDrive::get_y() const{
        return q.translation().y;
    }

    double DiffDrive::get_phi() const{
        return q.rotation();
    }

    WheelState DiffDrive::get_wheels() const{
        return w;
    }

    std::ostream & operator<<(std::ostream & os, const WheelState & ws){
        os << "left: " << ws.l << " right: " << ws.r;
        return os;
    }

    void DiffDrive::fk(double new_left, double new_right){
        // taken from MR textbook, eq 13.15
        const auto dphi = (radius/(track))*(-1.0*new_left + new_right);
        const auto dx = 0.5*radius*cos(q.rotation())*(new_left + new_right);
        const auto dy = 0.5*radius*sin(q.rotation())*(new_left + new_right);
        // update position of the wheels
        w.l += new_left;
        w.r += new_right;
        auto new_phi = q.rotation() + dphi;
        new_phi = normalize_angle(new_phi);
        // update configuration vector in world frame
        q = Transform2D(Vector2D{q.translation().x + dx,
                                 q.translation().y + dy},
                        new_phi);
    }

    WheelState DiffDrive::ik(Twist2D tw){
        if (almost_equal(tw.linear_velocity().y, 0)){
            // equation 7 and 8 in doc/Kinematics.pdf
            const auto phi_l = (1./radius)*(tw.linear_velocity().x - 0.5*track*tw.angular_velocity());
            const auto phi_r = (1./radius)*(tw.linear_velocity().x + 0.5*track*tw.angular_velocity());
            // don't update anything, just return the required wheel controls.
            return WheelState{phi_l,phi_r};
        } else {
            throw std::logic_error("Y component of control twist must be 0!");
        }
    }

    void DiffDrive::change_state(Transform2D config){
        q = config;
    }
}