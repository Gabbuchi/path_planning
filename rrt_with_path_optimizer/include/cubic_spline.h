#ifndef CJL_CUBIC_SPLINE_H
#define CJL_CUBIC_SPLINE_H

#include <vector>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Path{
private:
    vector<double> s_, s_v_, s_a_, a_jerk_;//quartic_polynomial为采样的s，s就是cubicspline的s的一部分
    vector<double> l_, l_v_, l_a_, l_jerk_;//quintic_polynomial
    vector<double> x_, y_, yaw_, curve_; //cubic_spline
public:
    Path();
    ~Path(){}

};


class CubicSpline{
private:
    vector<double> x_,y_;
    vector<double> a_,b_,c_,d_;
    vector<double> h_;
    int nx_;
public:
    CubicSpline(vector<double> x,vector<double> y);
    CubicSpline(){}
    ~CubicSpline(){}
    void get_step(const vector<double>& x);
    MatrixXd create_A();
    VectorXd create_B();
    double get_position(double x);
    double get_first_derivative(double x);
    double get_second_derivative(double x);
};

class CubicSpline2D{
private:
    vector<double> s_;
    CubicSpline sx_;
    CubicSpline sy_;
public:
    CubicSpline2D(vector<double> x, vector<double> y);
    CubicSpline2D(){}
    ~CubicSpline2D(){}
    void calc_s(vector<double> x,vector<double> y);
    Vector2d get_position(double s);
    vector<double> get_s();
    double calc_yaw(double s);
    double calc_curvature(double s);
    static vector<vector<double>> calc_spline_path(vector<double> x,vector<double> y,double ds = 0.1);
    vector<vector<double>> calc_path_with_xy_yaw_curv(double ds = 0.1);
};


#endif // CJL_CUBIC_SPLINE_H