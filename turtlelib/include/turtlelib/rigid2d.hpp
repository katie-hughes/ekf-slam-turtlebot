#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
    if (d1<d2){
        return (d2-d1)<epsilon;
    } else {
        return (d1-d2)<epsilon;
    }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
    return deg*(PI/180.);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
    return rad*(180./PI);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief put an angle into the (-pi,pi] range.
    /// rad - incoming angle in radians
    /// \return Rescaled angle in radians
    double normalize_angle(double rad);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
        
        /// \brief add a vector
        /// \param rhs - the vector to add
        /// \return a reference to the newly transformed vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract a vector
        /// \param rhs - the vector to subtract
        /// \return a reference to the newly transformed vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief scale the vector by a constant.
        /// \param scalar - the scaling factor
        /// \return a reference to the newly transformed vector
        Vector2D & operator*=(const double & scalar);
    };

    /// \brief Add two vectors together.
    /// \param lhs - the original vector
    /// \param rhs - vector to add
    /// \return the transformed vector
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief Subtract one vector from another
    /// \param lhs - the original vector
    /// \param rhs - vector to subtract
    /// \return the transformed vector
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply a vector by a scalar.
    /// \param vec - the vector
    /// \param scalar - the scaling factoe
    /// \return the rescaled vector
    Vector2D operator*(Vector2D vec, const double & scalar);

    /// \brief Compute the dot product of two vectors
    /// \param v1 - First vector
    /// \param v2 - Second vector
    /// \return the dot product
    double dot(Vector2D v1, Vector2D v2);

    /// \brief Compute the magnitude of a vector
    /// \param vec - the vector
    /// \return the magnitude
    double magnitude(Vector2D vec);

    /// \brief Get the angle between two vectors
    /// \param v1 - First vector
    /// \param v2 - Second vector
    /// \return the angle in radians between v1 and v2
    double angle(Vector2D v1, Vector2D v2);

    /// \brief normalize a 2 dimensional vector
    /// v - the vector to normalize
    /// \return A new normalized Vector2D
    Vector2D normalize(Vector2D v);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);



    /// \brief a twist
    class Twist2D
    {
    private:
        double angular {};
        Vector2D linear {};
    public:
        /// \brief Create an identity transformation
        Twist2D();

        /// \brief create a twist that is pure angular velocity
        /// \param w - angle of the rotation, in radians/s
        explicit Twist2D(double w);

        /// \brief create a twist that is pure linear velocity
        /// \param v - the linear velocity vector, in m/s
        explicit Twist2D(Vector2D v);

        /// \brief Create a generic twist
        /// \param w - the angular velocity vector, in radians/s
        /// \param v - the linear velocity vector, in m/s
        Twist2D(double w, Vector2D v);

        /// \brief get the angular component of the twist
        /// \return the angular velocity, in radians/s
        double angular_velocity() const;

        /// \brief the translational component of the twist
        /// \return the x,y translation
        Vector2D linear_velocity() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// deg: [1 2 3] for angular velocity 1, x velocity 2, y velocity 3
    /// \param os - an output stream
    /// \param tw - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Twist2D & tw);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        Vector2D linear {};
        double angular {};
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D
        /// \param tw - the twist to transform
        /// \return a Twist2D in the new coordinate system
        Twist2D operator()(Twist2D tw) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);



    /// \brief computes transform for a rigid body following a constant twist
    // for a unit time (in the body frame)
    /// \param tw - The twist to follow
    /// \return the transform that follows the twist for unit time
    Transform2D integrate_twist(Twist2D tw);


    /// @brief Calculate Euclidean distance between points (x1, y1) and (x2, y2)
    /// @param x1 first x coordinate
    /// @param y1 first y coordinate
    /// @param x2 second x coordinate
    /// @param y2 second y coordinate
    /// @return Euclidean distance
    double distance(double x1, double y1, double x2, double y2);

    /// @brief Calculate euclidean distance between two 2D points
    /// @param v1 point 1
    /// @param v2 point 2
    /// @return Euclidiean distance
    double distance(Vector2D v1, Vector2D v2);

}

#endif
