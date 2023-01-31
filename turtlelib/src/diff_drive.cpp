#include <iostream>
#include <cmath> 
#include <string>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive():
        w{WheelState{0.0,0.0}},
        q{Transform2D()},
        track{0.16},
        radius{0.033}
    {}

    DiffDrive::DiffDrive(Transform2D config):
        w{WheelState{0.0,0.0}},
        q{config},
        track{0.16},
        radius{0.033}
    {}

    Transform2D DiffDrive::get_config(){
        return q;
    }
}