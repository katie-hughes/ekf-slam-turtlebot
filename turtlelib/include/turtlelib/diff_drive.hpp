#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diff Drive library.


#include<iosfwd> // contains forward definitions for iostream objects
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    /// @brief Keeps track of current wheel angles of a diff drive robot.
    struct WheelState
    {   
        /// @brief the state of the left wheel, in radians
        double l = 0.0;
        /// @brief the state of the right wheel, in radians
        double r = 0.0;
    };

    /// @brief Maintain and update the state of a diff drive robot
    class DiffDrive
    {
    private:
        /// @brief current angles of the wheels in radians
        WheelState w;
        /// @brief current configuration of the robot in world frame
        Transform2D q;
        /// @brief physical parameters for the wheels in m
        double track, radius;
    public:
        /// @brief Default constructor to create diff drive object.
        /// Origin and wheel angles are 0.
        /// @param wheel_track distance from center of robot to wheel joint (m)
        /// @param wheel_radius radius of wheel (m)
        DiffDrive(double wheel_track, double wheel_radius);

        /// @brief Create instance where origin is not 0.
        /// @param config - starting location in world frame.
        /// @param wheel_track distance from center of robot to wheel joint (m)
        /// @param wheel_radius radius of wheel (m)
        DiffDrive(Transform2D config, double wheel_track, double wheel_radius);

        /// @brief the current configuration of the robot in the world frame
        /// @return the transform from world to robot.
        Transform2D get_config() const;

        /// @brief get x component of the configuration
        /// @return x coordinate in m
        double get_x() const;

        /// @brief get y component of the configuration
        /// @return y coordinate in m
        double get_y() const;

        /// @brief get phi of configuration
        /// @return phi in radians
        double get_phi() const;

        /// @brief Get current wheel state of configuration
        /// @return wheel state of current config in radians
        WheelState get_wheels() const;

        /// @brief Update the configuration based on new wheel positions
        /// @param new_left - new left wheel angle in radians
        /// @param new_right - new right wheel angle in radians
        void fk(double new_left, double new_right);

        /// @brief compute wheel velocities required to move at twist
        /// @param tw - the twist we want to move at
        /// @return - Wheel velocities required in rad/s
        WheelState ik(Twist2D tw);

        /// @brief change the current configuration of the robot.
        /// @param config desired x, y, and theta orientation of the robot.
        void change_state(Transform2D config);
    };

    /// @brief Output wheel state as "left: xx right: xx"
    /// @param os stream to output to
    /// @param ws wheel state to print
    std::ostream & operator<<(std::ostream & os, const WheelState & ws);

}


#endif
