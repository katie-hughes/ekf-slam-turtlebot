#include <iostream>
#include "rigid2d.hpp"
using namespace turtlelib;

int main(){
    Vector2D v_b = {};
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    // Read in deg, x, y
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    // Read in deg, x, y
    // print Tab
    // print Tba
    // print Tbc
    // print Tcb
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