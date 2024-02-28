#ifndef CJL_CUBIC_SPLINE_H
#define CJL_CUBIC_SPLINE_H

#include <vector>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

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
};

#endif // CJL_CUBIC_SPLINE_H