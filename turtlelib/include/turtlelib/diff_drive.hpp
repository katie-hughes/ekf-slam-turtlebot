#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diff Drive library.


#include<iosfwd> // contains forward definitions for iostream objects
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    class DiffDrive
    {
        double phi_left, phi_right;
        // This is the transform in the world frame
        Transform2D q;
        // Also probably need to specify some physical parameters
        double wheel_track, wheel_radius;
    public:
        /// \brief Create an identity transformation
        DiffDrive();

        // DiffDrive(Transform2D config);

    };

}

#endif
