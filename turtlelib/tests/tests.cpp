#include "turtlelib/rigid2d.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE( "Factorials are computed", "[factorial]" ) {
    turtlelib::Transform2D Ttest = {turtlelib::Vector2D{2,3}, 45};
    REQUIRE( 1  == 1 );
    REQUIRE( Ttest.rotation() == 45);
}