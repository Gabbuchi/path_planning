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


struct Path{
    vector<double> t_;
    vector<double> s_, s_v_, s_a_, a_jerk_;//quartic_polynomial为采样的s，s就是cubicspline的s的一部分
    vector<double> l_, l_v_, l_a_, l_jerk_;//quintic_polynomial
    vector<double> x_, y_, yaw_, ds_, kappa_; //cubic_spline的s结合l推算

    vector<vector<double>> SL_2_XY(CubicSpline2D& reference_line){
        x_.clear();
        y_.clear();
        for(size_t i = 0; i < s_.size(); i++){
            if(s_[i] > reference_line.get_s().back()){
                break;
            }
            Vector2d xy_ref = reference_line.get_position(s_[i]);
            double yaw = reference_line.calc_yaw(s_[i]);
            x_.push_back(xy_ref.coeff(0) + l_[i] * cos(yaw + 3.14159265/2));
            y_.push_back(xy_ref.coeff(1) + l_[i] * sin(yaw + 3.14159265/2));
        }
    }

    void calc_yaw_curv(){
        yaw_.clear();
        yaw_.resize(x_.size());
        kappa_.clear();
        kappa_.resize(x_.size());
        ds_.clear();
        ds_.resize(x_.size());
        for(size_t i = 0; i < x_.size() - 1; i++){
            double dx = x_[i+1] - x_[i];
            double dy = y_[i+1] - y_[i];
            ds_.push_back(hypot(dx,dy));
            yaw_.push_back(atan2(dy,dx));
        }
        if(yaw_.empty()) return;
        yaw_.push_back(yaw_.back());
        ds_.push_back(ds_.back());

        for (size_t i = 0; i< yaw_.size()-1; i++) {
            kappa_.push_back((yaw_[i + 1] - yaw_[i]) / ds_[i]);
        }
    }
};


#endif // CJL_CUBIC_SPLINE_H