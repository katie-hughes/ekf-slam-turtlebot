#include <iostream>
#include "rigid2d.hpp"
using namespace turtlelib;

int main(){
    Transform2D Tab = {}; //{Vector2D{2,3}, 20};
    Transform2D Tbc = {};
    Vector2D v_b = {};
    Twist2D V_b = {};
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tbc;
    // define Tac
    Transform2D Tac = Tab*Tbc;
    // print out Ts and their inverse
    std::cout << "Tab: " << Tab << std::endl;
    std::cout << "Tba: " << Tab.inv() << std::endl;
    std::cout << "Tbc: " << Tbc << std::endl;
    std::cout << "Tcb: " << Tbc.inv() << std::endl;
    std::cout << "Tac: " << Tac << std::endl;
    std::cout << "Tca: " << Tac.inv() << std::endl;

    // Vector2D
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    // print v_bhat
    std::cout << "v_a: " << Tab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << (Tbc.inv())(v_b) << std::endl;

    // Twist
    std::cout << "Enter twist V_b:" << std::endl;
    // enter 1 1 1
    // print V_a
    std::cout << "V_b: " << V_b << std::endl;
    // print V_c
    return 0;
}