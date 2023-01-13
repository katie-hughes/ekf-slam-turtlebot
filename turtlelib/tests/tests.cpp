#include "turtlelib/rigid2d.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE( "Rotation", "transform" ) {
    turtlelib::Transform2D Ttest1 = {turtlelib::Vector2D{2,3}, 45};
    turtlelib::Transform2D Ttest2 = {turtlelib::Vector2D{2,3}, -45};
    REQUIRE( 1  == 1 );
    REQUIRE( Ttest1.rotation() == 45);
    REQUIRE( Ttest2.rotation() == -45);
}