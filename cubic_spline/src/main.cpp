#include "../include/cubic_spline.h"
#include "../include/matplotlibcpp.h"
#include <iostream>
using namespace std;
namespace plt = matplotlibcpp;
int main(){
    vector<double> x = {0,1.3,2,2.5,4,5,6.4};
    vector<double> y = {0.5,1.8,1.5,1,1.2,1.4,4};
    // vector<double> x = {0,1,2,3,4,5,8,10};
    // vector<double> y = {0.5,1.5,0,2,1.2,1.4,1,1.5};
    CubicSpline my_spline(x,y);
    cout <<"after create spline" << endl;
    vector<double> sp_x;
    vector<double> sp_y;
    int size_x = x.size();
    std::cout << size_x << std::endl;
    for(double i = 0; i < x.back(); i += 0.1){
        sp_x.push_back(i);
        sp_y.push_back(my_spline.get_position(i));
    }
    plt::named_plot("Data points", x, y, "xb");
    plt::named_plot("Cubic spline interpolation", sp_x, sp_y, "r");
    plt::grid(true);
    plt::legend();
    plt::title("Cubic Spline");
    plt::show();
    return 0;

}