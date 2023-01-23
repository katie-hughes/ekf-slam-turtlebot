#include "turtlelib/rigid2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

TEST_CASE( "Empty Constructor", "[transform]" ) { // Hughes, Katie
    turtlelib::Transform2D Ttest;
    REQUIRE( Ttest.rotation() == 0.0);
    REQUIRE( Ttest.translation().x == 0.0);
    REQUIRE( Ttest.translation().y == 0.0);
}


TEST_CASE( "Translation Only Constructor", "[transform]" ) { // Hughes, Katie
    float my_x = 2;
    float my_y = 3;
    turtlelib::Transform2D Ttest(turtlelib::Vector2D{my_x, my_y});
    REQUIRE( Ttest.rotation() == 0.0);
    REQUIRE( Ttest.translation().x == my_x);
    REQUIRE( Ttest.translation().y == my_y);
}


TEST_CASE( "Rotation Only Constructor", "[transform]" ) { // Hughes, Katie
    float my_ang = turtlelib::PI;
    turtlelib::Transform2D Ttest(my_ang);
    REQUIRE( Ttest.rotation() == my_ang);
    REQUIRE( Ttest.translation().x == 0.0);
    REQUIRE( Ttest.translation().y == 0.0);
}

TEST_CASE( "Rotation and Translation", "[transform]" ) { // Hughes, Katie
    float my_x = 2.;
    float my_y = 3.; 
    float my_ang = turtlelib::PI;
    turtlelib::Transform2D Ttest(turtlelib::Vector2D{my_x,my_y}, my_ang);
    REQUIRE( Ttest.rotation() == my_ang);
    REQUIRE( Ttest.translation().x == my_x);
    REQUIRE( Ttest.translation().y == my_y);
}

TEST_CASE( "Inverse", "[transform]" ) { // Hughes, Katie
    float my_x = 0.;
    float my_y = 1.; 
    float my_ang = turtlelib::PI/2;
    turtlelib::Transform2D Ttest(turtlelib::Vector2D{my_x,my_y}, my_ang);
    turtlelib::Transform2D Ttest_inv = Ttest.inv();
    REQUIRE( (Ttest.inv()).rotation() == -my_ang);
    REQUIRE_THAT( Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT( Ttest_inv.translation().y, Catch::Matchers::WithinAbs( 0.0, 1e-5));
}