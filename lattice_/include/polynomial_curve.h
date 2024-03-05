#ifndef CJL_POLYNOMIAL_CURVE_H
#define CJL_POLYNOMIAL_CURVE_H


#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

class quarticPolynomial{
private:
    double a0,a1,a2,a3,a4;
public:
    //沿路径s的运动状态
    //四次多项式仅考虑沿s方向的位置及速度加速度随时间t的变化，保证了速度加速度的连续性
    quarticPolynomial(double s,double v_s,double a_s,double v_s_expected,double a_s_expected,double t){
        a0 = s;
        a1 = v_s;
        a2 = a_s / 2;
        Matrix2d A;
        A << 3*pow(t,2), 4*pow(t,3),
             6*t,        12*pow(t,2);
        Vector2d b;
        b << v_s_expected - a1 - 2*a2*t, a_s_expected - 2*a2;
        Vector2d x = A.colPivHouseholderQr().solve(b);
        a3 = x.coeff(0);
        a4 = x.coeff(1);
    }
    quarticPolynomial(){}
    ~quarticPolynomial(){}

    double calc_s(double t){
        return a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4);
    }

    double calc_v_s(double t){
        return a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3);
    }

    double calc_a_s(double t){
        return 2*a2 + 6*a3*t + 12*a4*pow(t,2);
    }

    double calc_jerk_s(double t){
        return 6*a3 + 24*a4*t;
    }
};



class quinticPolynomial{
private:
    double a0,a1,a2,a3,a4,a5;
public:
    quinticPolynomial(double l,double v_l,double a_l,double l_expected, double v_l_expected,double a_l_expected,double t){
        a0 = l;
        a1 = v_l;
        a2 = a_l / 2;
        Matrix3d A;
        A << pow(t,3),   pow(t,4),   pow(t,5),
             3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
             6*t,        12*pow(t,2),20*pow(t,3);
        Vector3d b;
        b << l_expected - a0 - a1*t - a2*pow(t,2),
             v_l_expected - a1 - 2*a2*t,
             a_l_expected - 2*a2;
        Vector3d x = A.colPivHouseholderQr().solve(b);
        a3 = x.coeff(0);
        a4 = x.coeff(1);
        a5 = x.coeff(2);
    }
    quinticPolynomial(){}
    ~quinticPolynomial(){}

    double calc_l(double t){
        return a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) +a5*pow(t,5);
    }

    double calc_v_l(double t){
        return a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    }

    double calc_a_l(double t){
        return 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);
    }

    double calc_jerk_l(double t){
        return 6*a3 + 24*a4*t + 60*a5*pow(t,2);
    }

};

#endif //CJL_POLYNOMIAL_CURVE_H