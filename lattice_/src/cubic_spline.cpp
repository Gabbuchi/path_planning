#include "../include/cubic_spline.h"
#include <iostream>
using namespace std;
using namespace Eigen;

//参考：https://zhuanlan.zhihu.com/p/62860859
// https://www.cnblogs.com/xpvincent/archive/2013/01/26/2878092.html
// 三对角矩阵求解：https://www.cnblogs.com/xpvincent/archive/2013/01/25/2877411.html
CubicSpline::CubicSpline(vector<double> x, vector<double> y){
    x_ = x;
    y_ = y;
    a_ = y;
    nx_ = x_.size();
    b_.resize(nx_ - 1);  
    c_.resize(nx_ - 1); 
    d_.resize(nx_ - 1);
    get_step(x_);
    MatrixXd A = create_A();
    cout << "after create A" << endl;
    VectorXd B = create_B();
    cout << "after create B" << endl;
    VectorXd m = A.colPivHouseholderQr().solve(B);
    cout << "after solve m" << endl;

    for(int i = 0; i < (nx_ - 1); i++){
        b_[i] = (y_[i+1] - y_[i])/h_[i] - h_[i] * m.coeff(i) / 2
                - h_[i]*(m.coeff(i+1) - m.coeff(i))/6;
        cout << "b_[" << i << "] = " << b_[i] << endl;
        c_[i] = m.coeff(i) / 2;
        d_[i] = (m.coeff(i+1) - m.coeff(i)) / (6 * h_[i]);
    }
    cout << "after create b,c,d" << endl;

}

void CubicSpline::get_step(const vector<double>& x){
    if(x.empty()){
        cout << "x 不能为空" << endl;
    }
    for(int i = 0; i < x.size() - 1; i++){
        h_.push_back(x[i+1] - x[i]);
    }
}

MatrixXd CubicSpline::create_A(){
    MatrixXd temp_A = MatrixXd::Zero(nx_,nx_);
    temp_A(0,0) = 1;
    temp_A(nx_-1,nx_-1) = 1;
    for(int i = 1; i < nx_ - 1; i++){
        temp_A(i,i) = 2*(h_[i-1]+h_[i]);
        temp_A(i,i-1) = h_[i-1];
        temp_A(i,i+1) = h_[i];
    }
    return temp_A;
}

VectorXd CubicSpline::create_B(){
    VectorXd temp_B = VectorXd::Zero(nx_);
    for(int i = 1; i < nx_ - 1; i++){
        temp_B(i) = 6*((y_[i+1] - y_[i])/h_[i] - 
                    (y_[i] - y_[i-1])/h_[i-1]);
    }
    return temp_B;
}

//重新写一遍
double CubicSpline::get_position(double x){
    if(x < x_.front() || x > x_.back()){ 
        cout <<"x out of range" << endl;
    }
    auto it = std::upper_bound(x_.begin(),x_.end(),x);
    int index = std::distance(x_.begin(),it) - 1;
    double delta_x = x - x_[index];
    return a_[index] + b_[index]*delta_x + c_[index]*pow(delta_x,2) + d_[index]*pow(delta_x,3);
}

double CubicSpline::get_first_derivative(double x){
    if(x < x_.front() || x > x_.back()){
        cout <<"x out of range" << endl;
    }
    auto it = upper_bound(x_.begin(),x_.end(),x);
    int index = std::distance(x_.begin(),it) - 1;
    double delta_x = x - x_[index];
    return b_[index] + 2*c_[index]*delta_x + 3*d_[index]*pow(delta_x,2);
}

double CubicSpline::get_second_derivative(double x){
    if(x < x_.front() || x > x_.back()){
        cout <<"x out of range" << endl;
    }
    auto it = std::upper_bound(x_.begin(),x_.end(),x);
    int index = std::distance(x_.begin(),it) - 1;
    double delta_x = x - x_[index];
    return 2*c_[index] + 6*d_[index]*delta_x;
}

//cubicSpline2D

CubicSpline2D::CubicSpline2D(vector<double> x,vector<double> y){
    calc_s(x,y);
    sx_ = CubicSpline(s_,x);
    sy_ = CubicSpline(s_,y);
}

void CubicSpline2D::calc_s(vector<double> x,vector<double> y){
    vector<double> diff_x,diff_y;
    for(size_t i = 0; i < x.size(); i++){
        diff_x.push_back(x[i+1] - x[i]);
        diff_y.push_back(y[i+1] - y[i]);
    }
    vector<double> ds;
    for(size_t i = 0; i < diff_x.size(); i++){
        ds.push_back(hypot(diff_x[i],diff_y[i]));
    }
    double count_ = 0;
    s_.push_back(count_);
    for(size_t i = 0; i < ds.size() - 1; i++){
        count_ += ds[i];
        s_.push_back(count_);
    }
}

Vector2d CubicSpline2D::get_position(double s){
    return {sx_.get_position(s), sy_.get_position(s)};
}

vector<double> CubicSpline2D::get_s(){
    return s_;
}

double CubicSpline2D::calc_yaw(double s){
    double dx_ds = sx_.get_first_derivative(s);
    double dy_ds = sy_.get_first_derivative(s);
    return atan2(dy_ds,dx_ds);
}
//k = (ddy * dx - ddx * dy) / pow(dx * dx + dy * dy, 1.50)
double CubicSpline2D::calc_curvature(double s){
    double dx = sx_.get_first_derivative(s);
    double dy = sy_.get_first_derivative(s);
    double ddx = sx_.get_second_derivative(s);
    double ddy = sy_.get_second_derivative(s);
    return (ddy * dx - ddx * dy) / pow(dx * dx + dy * dy, 1.50);
}

vector<vector<double>> CubicSpline2D::calc_spline_path(vector<double> x,vector<double> y,double ds){
    CubicSpline2D* sp_temp = new CubicSpline2D(x,y);
    vector<vector<double>> path(4); 
    for(double i = sp_temp->s_.front(); i < sp_temp->s_.back(); i+=ds){
        Vector2d ixy = sp_temp->get_position(i);
        path[0].push_back(ixy.coeff(0));
        path[1].push_back(ixy.coeff(1));
        path[2].push_back(sp_temp->calc_yaw(i));
        path[3].push_back(sp_temp->calc_curvature(i));
    }
    delete sp_temp;
    //[x, y, yaw, kappa]
    return path;
}

vector<vector<double>> CubicSpline2D::calc_path_with_xy_yaw_curv(double ds){
    vector<vector<double>> path(4); 
    for(double i = s_.front(); i < s_.back(); i+=ds){
        Vector2d ixy = get_position(i);
        path[0].push_back(ixy.coeff(0));
        path[1].push_back(ixy.coeff(1));
        path[2].push_back(calc_yaw(i));
        path[3].push_back(calc_curvature(i));
    }
    return path;
}
