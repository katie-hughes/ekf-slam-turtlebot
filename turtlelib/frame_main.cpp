#include <iostream>
#include "rigid2d.hpp"

int main(){
    turtlelib::Transform2D Tab = {}; //{Vector2D{2,3}, 20};
    turtlelib::Transform2D Tbc = {};
    turtlelib::Vector2D v_b = {};
    turtlelib::Twist2D V_b = {};
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    // Read in deg, x, y
    std::cin >> Tbc;
    // define Tac
    turtlelib::Transform2D Tac = Tab*Tbc;
    // print out Ts and their inverse
    // These are all right
    std::cout << "T_{a,b}: " << Tab << std::endl;
    std::cout << "T_{b,a}: " << Tab.inv() << std::endl;
    std::cout << "T_{b,c}: " << Tbc << std::endl;
    std::cout << "T_{c,b}: " << Tbc.inv() << std::endl;
    std::cout << "T_{a,c}: " << Tac << std::endl;
    std::cout << "T_{c,a}: " << Tac.inv() << std::endl;

    // Vector2D
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    // print out different transforms
    std::cout << "v_bhat: " << normalize(v_b) << std::endl;
    std::cout << "v_a: " << Tab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    // THIS ONE IS SLIGHTLY OFF WHYYY ;-;
    std::cout << "v_c: " << (Tbc.inv())(v_b) << std::endl;

    // Twist
    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;
    // print out different transforms
    std::cout << "V_a: " << Tab(V_b) << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << (Tbc.inv())(V_b) << std::endl;
    return 0;
}