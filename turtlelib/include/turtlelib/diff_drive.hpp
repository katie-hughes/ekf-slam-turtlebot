#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diff Drive library.


#include<iosfwd> // contains forward definitions for iostream objects
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    struct WheelState
    {   
        /// @brief the angle in radians of the left wheel
        double l = 0.0;
        /// @brief the angle in radians of the right wheel
        double r = 0.0;
    };

    class DiffDrive
    {
        WheelState w;
        // This is the transform in the world frame
        Transform2D q;
        // Also probably need to specify some physical parameters for wheels
        double track, radius;
    public:
        /// @brief Create instance where wheels/origin is 0.
        // default wheel_track and wheel_radius from turtlebot.
        DiffDrive();

        /// @brief Create instance where origin is not 0.
        /// @param config - starting location in world frame.
        DiffDrive(Transform2D config);

        /// @brief the current configuration of the robot in the world frame
        /// @return the transform from world to robot.
        Transform2D get_config();

        /// @brief Update the configuration based on new wheel positions
        /// @param new_left - new left wheel angle in radians
        /// @param new_right - new right wheel angle in radians
        void fk(double new_left, double new_right);

        WheelState ik(Twist2D tw);

    };

}

#endif
