#include "turtlelib/rigid2d.hpp"
#include "turtlelib/circles.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>


namespace turtlelib {

    TEST_CASE("detectCircle1", "circles"){
        std::vector<Vector2D> test1 = {Vector2D{1,7},
                                       Vector2D{2,6},
                                       Vector2D{5,8},
                                       Vector2D{7,7},
                                       Vector2D{9,5},
                                       Vector2D{3,7}};
        Circle desired_result1{4.615482,2.807354,4.8275};
        Circle real_result1 = detectCircle(test1);
        REQUIRE_THAT(real_result1.x, Catch::Matchers::WithinAbs(desired_result1.x, 1e-4));
        REQUIRE_THAT(real_result1.y, Catch::Matchers::WithinAbs(desired_result1.y, 1e-4));
        REQUIRE_THAT(real_result1.r, Catch::Matchers::WithinAbs(desired_result1.r, 1e-4));
    }

    TEST_CASE("detectCircle2", "circles"){
        std::vector<Vector2D> test2 = {Vector2D{-1,0},
                                       Vector2D{-0.3,-0.06},
                                       Vector2D{0.3,0.1},
                                       Vector2D{1,0}};
        Circle desired_result2{0.4908357, -22.15212, 22.17979};
        Circle real_result2 = detectCircle(test2);
        REQUIRE_THAT(real_result2.x, Catch::Matchers::WithinAbs(desired_result2.x, 1e-4));
        REQUIRE_THAT(real_result2.y, Catch::Matchers::WithinAbs(desired_result2.y, 1e-4));
        REQUIRE_THAT(real_result2.r, Catch::Matchers::WithinAbs(desired_result2.r, 1e-4));
    }


    TEST_CASE("isCircle1", "circles"){
        // i take these from starting clusters when running nuslam with no noise
        std::vector<Vector2D> test1 = {Vector2D{0.530956, 0.877961},
                                       Vector2D{0.501033, 0.862014},
                                       Vector2D{0.483308, 0.865862},
                                       Vector2D{0.469719, 0.877042},
                                       Vector2D{0.462, 0.899921}};

        std::vector<Vector2D> test2 = {Vector2D{-0.610432, -0.76346},
                                       Vector2D{-0.589061, -0.763608},
                                       Vector2D{-0.574354, -0.77196},
                                       Vector2D{-0.564374, -0.786778}};

        std::vector<Vector2D> test3 = {Vector2D{0.662758, -0.70755},
                                       Vector2D{0.664875, -0.685499},
                                       Vector2D{0.674604, -0.671733},
                                       Vector2D{0.689829, -0.663387},
                                       Vector2D{0.717297, -0.666165}};
        REQUIRE(isCircle(test1));
        REQUIRE(isCircle(test2));
        REQUIRE(isCircle(test3));
    }

}