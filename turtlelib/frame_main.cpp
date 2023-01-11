#include <iostream>
#include "rigid2d.hpp"
using namespace turtlelib;

int main(){
    Vector2D v_b = {};
    Transform2D Tab = {}; //{Vector2D{2,3}, 20};
    Transform2D Tbc = {};
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tbc;
    std::cout << "Tab: " << Tab << std::endl;
    std::cout << "Tba: " << Tab.inv() << std::endl;
    std::cout << "Tbc: " << Tbc << std::endl;
    std::cout << "Tcb: " << Tbc.inv() << std::endl;
    // print Tac
    // print Tca
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    // enter 1 1
    // print v_bhat
    // print v_a
    std::cout << "v_b:" << v_b << std::endl;
    // print v_c
    std::cout << "Enter twist V_b:" << std::endl;
    // enter 1 1 1
    // print V_a
    // print V_b
    // print V_c
    return 0;
}