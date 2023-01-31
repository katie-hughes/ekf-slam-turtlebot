#include "turtlelib/rigid2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>

namespace turtlelib
{

    TEST_CASE("Empty Constructor", "[transform]")
    { // Hughes, Katie
        Transform2D Ttest;
        REQUIRE(Ttest.rotation() == 0.0);
        REQUIRE(Ttest.translation().x == 0.0);
        REQUIRE(Ttest.translation().y == 0.0);
    }

    TEST_CASE("Translation Only Constructor", "[transform]")
    { // Hughes, Katie
        float my_x = 2;
        float my_y = 3;
        Transform2D Ttest(Vector2D{my_x, my_y});
        REQUIRE(Ttest.rotation() == 0.0);
        REQUIRE(Ttest.translation().x == my_x);
        REQUIRE(Ttest.translation().y == my_y);
    }

    TEST_CASE("Rotation Only Constructor", "[transform]")
    { // Hughes, Katie
        float my_ang = PI;
        Transform2D Ttest(my_ang);
        REQUIRE(Ttest.rotation() == my_ang);
        REQUIRE(Ttest.translation().x == 0.0);
        REQUIRE(Ttest.translation().y == 0.0);
    }

    TEST_CASE("Rotation and Translation", "[transform]")
    { // Hughes, Katie
        float my_x = 2.;
        float my_y = 3.;
        float my_ang = PI;
        Transform2D Ttest(Vector2D{my_x, my_y}, my_ang);
        REQUIRE(Ttest.rotation() == my_ang);
        REQUIRE(Ttest.translation().x == my_x);
        REQUIRE(Ttest.translation().y == my_y);
    }

    TEST_CASE("operator()(Vector2D v)", "[transform]") // Marno, Nel
    {
        double test_rot = PI / 2.0;
        double test_x = 0.0;
        double test_y = 1.0;
        Transform2D T_ab{{test_x, test_y}, test_rot};
        Vector2D v_b{1, 1};
        Vector2D v_a = T_ab(v_b);
        REQUIRE(almost_equal(v_a.x, -1.0));
        REQUIRE(almost_equal(v_a.y, 2.0));
    }

    TEST_CASE("operator()(Twist2D t)", "[transform]") // Marno, Nel
    {
        double test_rot = PI / 2.0;
        double test_x = 0.0;
        double test_y = 1.0;
        Transform2D T_ab{{test_x, test_y}, test_rot};
        Twist2D V_b{1, Vector2D{1, 1}};
        Twist2D V_a = T_ab(V_b);
        REQUIRE(almost_equal(V_a.angular_velocity(), 1.0));
        REQUIRE(almost_equal(V_a.linear_velocity().x, 0.0));
        REQUIRE(almost_equal(V_a.linear_velocity().y, 1.0));
    }

    TEST_CASE("Inverse", "[transform]")
    { // Hughes, Katie
        float my_x = 0.;
        float my_y = 1.;
        float my_ang = PI / 2.;
        Transform2D Ttest(Vector2D{my_x, my_y}, my_ang);
        Transform2D Ttest_inv = Ttest.inv();
        REQUIRE((Ttest.inv()).rotation() == -my_ang);
        REQUIRE_THAT(Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
        REQUIRE_THAT(Ttest_inv.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("operator *=", "[transform]")
    { // Megan, Sindelar
        Vector2D trans_ab = {1, 2};
        double rotate_ab = 0;
        Transform2D T_ab_1 = {trans_ab, rotate_ab}; // T_ab's are all the same,
        Transform2D T_ab_2 = {trans_ab, rotate_ab}; // but, need different vars
        Transform2D T_ab_3 = {trans_ab, rotate_ab}; // b/c getting overwritten otherwise
        Vector2D trans_bc = {3, 4};
        double rotate_bc = PI / 2;
        Transform2D T_bc = {trans_bc, rotate_bc};
        REQUIRE_THAT((T_ab_1 *= T_bc).translation().x, Catch::Matchers::WithinAbs(4.0, 1e-5));
        REQUIRE_THAT((T_ab_2 *= T_bc).translation().y, Catch::Matchers::WithinAbs(6.0, 1e-5));
        REQUIRE_THAT((T_ab_3 *= T_bc).rotation(), Catch::Matchers::WithinAbs(PI / 2, 1e-5));
    }

    TEST_CASE("Stream insertion operator <<", "[transform]") // Ava, Zahedi
    {
        Vector2D vec;
        vec.x = 1.0;
        vec.y = 3.4;
        double phi = 0.0;
        Transform2D tf(vec, phi);
        std::string str = "deg: 0 x: 1 y: 3.4";
        std::stringstream sstr;
        sstr << tf;
        REQUIRE(sstr.str() == str);
    }

    TEST_CASE("Stream extraction operator >>", "[transform]") // Ava, Zahedi
    {
        Transform2D tf;
        std::stringstream sstr;
        sstr << "deg: 90 x: 1 y: 3.4";
        sstr >> tf;
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(deg2rad(90), 1e-5));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
    }

    TEST_CASE("Normalize angle", "rigid2d") // Hughes, Katie
    {
        REQUIRE(normalize_angle(0) == 0);
        REQUIRE(normalize_angle(PI) == PI);
        REQUIRE_THAT(normalize_angle(-1 * PI), Catch::Matchers::WithinAbs(PI, 1e-5));

        REQUIRE(normalize_angle(-0.25 * PI) == -0.25 * PI);
        REQUIRE_THAT(normalize_angle(1.5 * PI), Catch::Matchers::WithinAbs(-0.5 * PI, 1e-5));
        REQUIRE_THAT(normalize_angle(-2.5 * PI), Catch::Matchers::WithinAbs(-0.5 * PI, 1e-5));

        REQUIRE_THAT(normalize_angle(2 * PI), Catch::Matchers::WithinAbs(0, 1e-5));
        REQUIRE_THAT(normalize_angle(-2 * PI), Catch::Matchers::WithinAbs(0, 1e-5));
        REQUIRE_THAT(normalize_angle(4 * PI), Catch::Matchers::WithinAbs(0, 1e-5));
        REQUIRE_THAT(normalize_angle(-4 * PI), Catch::Matchers::WithinAbs(0, 1e-5));
        // REQUIRE_THAT(normalize_angle(101*PI), Catch::Matchers::WithinAbs(PI, 1e-5));
        // REQUIRE_THAT(normalize_angle(-101*PI), Catch::Matchers::WithinAbs(PI, 1e-5));
        REQUIRE_THAT(normalize_angle(5 * PI), Catch::Matchers::WithinAbs(PI, 1e-5));
        REQUIRE_THAT(normalize_angle(-5 * PI), Catch::Matchers::WithinAbs(PI, 1e-5));
    }

    TEST_CASE("operator*", "Vector2D")
    {
        Vector2D vec = Vector2D{1, 2};
        Vector2D scaled = vec * 2.0;
        REQUIRE_THAT(scaled.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
        REQUIRE_THAT(scaled.y, Catch::Matchers::WithinAbs(4.0, 1e-5));
    }

    TEST_CASE("operator+", "Vector2D")
    {
        Vector2D vec1 = Vector2D{1, 2};
        Vector2D vec2 = Vector2D{3, 4};
        Vector2D sum = vec1 + vec2;
        REQUIRE_THAT(sum.x, Catch::Matchers::WithinAbs(4.0, 1e-5));
        REQUIRE_THAT(sum.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
    }

    TEST_CASE("operator-", "Vector2D")
    {
        Vector2D vec1 = Vector2D{10, 11};
        Vector2D vec2 = Vector2D{2, 5};
        Vector2D diff = vec1 - vec2;
        REQUIRE_THAT(diff.x, Catch::Matchers::WithinAbs(8.0, 1e-5));
        REQUIRE_THAT(diff.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
    }

    TEST_CASE("dot", "Vector2D")
    {
        Vector2D vec1 = Vector2D{1, 2};
        Vector2D vec2 = Vector2D{3, 4};
        // 1*3 + 2*4 = 3 + 8 = 11
        REQUIRE_THAT(dot(vec1, vec2), Catch::Matchers::WithinAbs(11.0, 1e-5));
    }

    TEST_CASE("magnitude", "Vector2D")
    {
        Vector2D vec1 = Vector2D{0, 2};
        Vector2D vec2 = Vector2D{-1, 0};
        REQUIRE_THAT(magnitude(vec1), Catch::Matchers::WithinAbs(2.0, 1e-5));
        REQUIRE_THAT(magnitude(vec2), Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

    TEST_CASE("angle", "Vector2D")
    {
        Vector2D vec1 = Vector2D{0, 2};
        Vector2D vec2 = Vector2D{1, 0};
        REQUIRE_THAT(angle(vec1, vec2), Catch::Matchers::WithinAbs(0.5*PI, 1e-5));
        REQUIRE_THAT(angle(vec2, vec1), Catch::Matchers::WithinAbs(-0.5*PI, 1e-5));
    }


}
