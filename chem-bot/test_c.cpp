#include <iostream>
using namespace std;
string(the_value) = "three";
string(a) = "";

struct Speeds
{
    int s1;
    int s2;
    int s_t;
};

int return_values(Speeds byval) {
    std::cout << "speed1:   " << byval.s1 << "\n";
    std::cout << "speed2:  " << byval.s2 << "\n";
    std::cout << "time: " << byval.s_t << "\n";
    return(1);
}

int main() 
{
    Speeds one = { 255, 200, 1000 };
    Speeds two = { 128, 128, 1000 };
    Speeds three = { 0, 255, 1000 };
    a = the_value;
    cout << "Displaying Information," << endl;
    
    // this works and compiles:
    return_values(three);
}