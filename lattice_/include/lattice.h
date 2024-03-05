#ifndef CJL_LATTICE_H
#define CJL_LATTICE_H

#include "cubic_spline.h"
#include "polynomial_curve.h"

using namespace std;
using namespace Eigen;


class Path{
public:
    double cost_;
    vector<double> t_,ds_;
    vector<double> s_, s_v_, s_a_, s_jerk_;//quartic_polynomial为采样的s，s就是cubicspline的s的一部分
    vector<double> l_, l_v_, l_a_, l_jerk_;//quintic_polynomial
    vector<double> x_, y_, yaw_, kappa_; //cubic_spline的s结合l推算

    void SL_2_XY(CubicSpline2D& reference_line);
    void calc_yaw_kappa();
    vector<vector<double>> get_path_info();

    //常量成员函数，表示不能修改对象的任何数据
    bool operator<(const Path& other) const{
        return cost_ > other.cost_;
    }
};


class lattice_planner{
private:
    vector<Path> paths_;
    Path optimal_path_;
public:
    lattice_planner(double l0, double v_l0, double a_l0, double s0, double v_s0, double a_s0,
                             CubicSpline2D& reference_line, vector<double> vehicle_config,const vector<vector<double>>& obs);

    void path_sampling(double l0, double v_l0, double a_l0, double s0, double v_s0, double a_s0,
                             CubicSpline2D& reference_line, vector<double> vehicle_config,const vector<vector<double>>& obs);
    
    void extract_optimal_path(const vector<vector<double>>& obs);

    vector<Path> get_paths(){ return paths_; }
    Path get_optimal_path(){ return optimal_path_; }

};


#endif //CJL_LATTICE_H