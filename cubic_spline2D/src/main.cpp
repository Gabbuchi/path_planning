#include "../include/cubic_spline.h"
#include "../include/matplotlibcpp.h"
#include "../include/polynomial_curve.h"
#include <iostream>
using namespace std;
namespace plt = matplotlibcpp;
int main(){
    vector<double> x = {0,1.3,2,2.5,4,5,6.4};
    vector<double> y = {0.5,1.8,1.5,1,1.2,1.4,4};
    // vector<double> x = {1,2,3,4,5,6,5,4.5,4,3,2,1.5,0.5,1};
    // vector<double> y = {1,0.8,0.75,0.7,1,2.5,4,4.5,5,7,5,4.5,2.5,1};
    // vector<double> x = {0,1.3,2,2.5,4,5,6,7};
    // vector<double> y = {0.5,1.8,1.5,1,1.2,1.4,4,3};
    // vector<double> x = {0,1,2,3,4,5,6,9};
    // vector<double> y = {0.5,1.8,1.5,1,1.2,1.4,2,1.3};
    CubicSpline my_spline(x,y);
    cout <<"after create spline" << endl;
    vector<double> sp_x;
    vector<double> sp_y;
    for(double i = 0; i < x.back(); i += 0.1){
        sp_x.push_back(i);
        sp_y.push_back(my_spline.get_position(i));
    }

    // vector<double> sp2d_x;
    // vector<double> sp2d_y;
    // CubicSpline2D my_spline_2d(x,y);
    // cout <<"after create spline2d" << endl;
    // vector<double> s = my_spline_2d.get_s();
    // for(double i = s.front(); i < s.back(); i += 0.01){
    //     sp2d_x.push_back(my_spline_2d.get_position(i).coeff(0));
    //     sp2d_y.push_back(my_spline_2d.get_position(i).coeff(1));
    // }

    CubicSpline2D my_spline2D(x,y);
    vector<vector<double>> sp_path = my_spline2D.calc_path_with_xy_yaw_curv(0.1);

    // vector<vector<double>> sp_path = CubicSpline2D::calc_spline_path(x,y);
    // cout << "after create sp_path" << endl;

    vector<double> quartic_x;
    vector<double> quartic_y;
    quarticPolynomial quartic_curve(0,1,0,1,0,3);
    //(double s,double v_s,double a_s,double v_s_expected,double a_s_expected,double t)
    for(double i = 0; i < 3.1; i+= 0.1){
        quartic_x.push_back(i);
        quartic_y.push_back(quartic_curve.calc_s(i));
    }

    vector<double> quintic_x;
    vector<double> quintic_y;
    quinticPolynomial quintic_curve(0,0,5,5,3,1,3);
    //double l,double v_l,double a_l,double l_expected, double v_l_expected,double a_l_expected,double t
    for(double i = 0; i < 3.1; i+= 0.1){
        quartic_x.push_back(i);
        quartic_y.push_back(quintic_curve.calc_l(i));
    }

    plt::named_plot("Data points", x, y, "xb");
    plt::named_plot("Cubic spline interpolation", sp_x, sp_y, "r");
    //plt::named_plot("Cubic spline2D interpolation", sp2d_x, sp2d_y, "g");
    plt::named_plot("Cubic spline2D interpolation", sp_path[0], sp_path[1], "g");
    //plt::named_plot("quartic polynomial interpolation", quartic_x, quartic_y, "b");
    //plt::named_plot("quintic polynomial interpolation", quintic_x, quintic_y, "y");
    plt::grid(true);
    plt::legend();
    plt::title("Cubic Spline");
    plt::show();
    return 0;

}