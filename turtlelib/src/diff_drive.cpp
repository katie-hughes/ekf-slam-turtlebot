#include <iostream>
#include <cmath> 
#include <string>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive():
        phi_left{0},
        phi_right{0},
        q{Transform2D()},
        wheel_track{0.16},
        wheel_radius{0.033}
    {}
}