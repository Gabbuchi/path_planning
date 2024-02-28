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
    if(x < x_[0] || x > x_.back()){
        throw std::invalid_argument("x out of range");
    }
    auto it = std::upper_bound(x_.begin(),x_.end(),x);
    int index = std::distance(x_.begin(),it) - 1;
    double delta_x = x - x_[index];
    return a_[index] + b_[index]*delta_x + c_[index]*pow(delta_x,2) + d_[index]*pow(delta_x,3);
}