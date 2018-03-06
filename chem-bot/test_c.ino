#include <iostream>
using namespace std;
int the_value;

struct Speeds
{
    int s1;
    int s2;
    int s_t;
};

void fill_struct() {
    Speeds one;
    one.s1 = 255;
    one.s2 = 200;
    one.s_t = 1000;

    Speeds two;
    two.s1 = 255;
    two.s2 = 128;
    two.s_t = 1500;

    Speeds three;
    three.s1 = 255;
    three.s2 = 128;
    three.s_t = 1500;
}

int main() 
{
    fill_struct();
    cout << "\nDisplaying Information," << endl;
    cout << "Speed 1: " << (the_value).s1 << endl;
    cout << "Speed 2: " << (the_value).s2 << endl;
    cout << "Speed 3: " << (the_value).s_t << endl;
}