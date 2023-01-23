#include "turtlelib/rigid2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>


namespace turtlelib{


TEST_CASE( "Empty Constructor", "[transform]" ) { // Hughes, Katie
    Transform2D Ttest;
    REQUIRE( Ttest.rotation() == 0.0);
    REQUIRE( Ttest.translation().x == 0.0);
    REQUIRE( Ttest.translation().y == 0.0);
}


TEST_CASE( "Translation Only Constructor", "[transform]" ) { // Hughes, Katie
    float my_x = 2;
    float my_y = 3;
    Transform2D Ttest(Vector2D{my_x, my_y});
    REQUIRE( Ttest.rotation() == 0.0);
    REQUIRE( Ttest.translation().x == my_x);
    REQUIRE( Ttest.translation().y == my_y);
}


TEST_CASE( "Rotation Only Constructor", "[transform]" ) { // Hughes, Katie
    float my_ang = PI;
    Transform2D Ttest(my_ang);
    REQUIRE( Ttest.rotation() == my_ang);
    REQUIRE( Ttest.translation().x == 0.0);
    REQUIRE( Ttest.translation().y == 0.0);
}

TEST_CASE( "Rotation and Translation", "[transform]" ) { // Hughes, Katie
    float my_x = 2.;
    float my_y = 3.; 
    float my_ang = PI;
    Transform2D Ttest(Vector2D{my_x,my_y}, my_ang);
    REQUIRE( Ttest.rotation() == my_ang);
    REQUIRE( Ttest.translation().x == my_x);
    REQUIRE( Ttest.translation().y == my_y);
}

TEST_CASE("operator()(Vector2D v)","[transform]") // Marno, Nel
{
   double test_rot = PI/2.0;
   double test_x = 0.0;
   double test_y = 1.0;
   Transform2D T_ab{{test_x,test_y}, test_rot};
   Vector2D v_b{1, 1};
   Vector2D v_a = T_ab(v_b);
   REQUIRE(almost_equal(v_a.x, -1.0));
   REQUIRE(almost_equal(v_a.y, 2.0));
}


TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel
{
   double test_rot = PI/2.0;
   double test_x = 0.0;
   double test_y = 1.0;
   Transform2D T_ab{{test_x,test_y}, test_rot};
   Twist2D V_b{1, Vector2D{1, 1}};
   Twist2D V_a = T_ab(V_b);
   REQUIRE(almost_equal(V_a.angular_velocity(), 1.0));
   REQUIRE(almost_equal(V_a.linear_velocity().x, 0.0));
   REQUIRE(almost_equal(V_a.linear_velocity().y, 1.0));
}

TEST_CASE( "Inverse", "[transform]" ) { // Hughes, Katie
    float my_x = 0.;
    float my_y = 1.; 
    float my_ang = PI/2.;
    Transform2D Ttest(Vector2D{my_x,my_y}, my_ang);
    Transform2D Ttest_inv = Ttest.inv();
    REQUIRE( (Ttest.inv()).rotation() == -my_ang);
    REQUIRE_THAT( Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT( Ttest_inv.translation().y, Catch::Matchers::WithinAbs( 0.0, 1e-5));
}

TEST_CASE( "Stream insertion operator <<", "[transform]" ) // Ava, Zahedi
{
   Vector2D vec;
   vec.x = 1.0;
   vec.y = 3.4;
   double phi = 0.0;
   Transform2D tf(vec, phi);
   std::string str = "deg: 0 x: 1 y: 3.4";
   std::stringstream sstr;
   sstr << tf;
   REQUIRE( sstr.str() == str );
}

TEST_CASE( "Stream extraction operator >>", "[transform]" ) // Ava, Zahedi
{
   Transform2D tf;
   std::stringstream sstr;
   sstr << "deg: 90 x: 1 y: 3.4";
   sstr >> tf;
   REQUIRE_THAT( tf.rotation(),      Catch::Matchers::WithinAbs(deg2rad(90), 1e-5));
   REQUIRE_THAT( tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
   REQUIRE_THAT( tf.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
}


}
