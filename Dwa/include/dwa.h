#ifndef CJL_DWA_H
#define CJL_DWA_H

#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cmath>

#define PI 3.14159265

struct Control{
    double v;
    double w;
    Control() : v(0), w(0){}
    Control(double v, double w) : v(v), w(w){}
};

struct State{
    double x;
    double y;
    double yaw;
    double v;
    double w;
    State(double x, double y, double yaw, double v, double w) :
        x(x), y(y), yaw(yaw), v(v), w(w){}
};

struct Window{
    double v_min;
    double v_max;
    double w_min;
    double w_max; //角速度限制可以加上最大曲率半径限制，本项目未考虑
    Window() : v_min(0.0), v_max(0.0), w_min(0.0), w_max(0.0) {}
    Window(double v_min, double v_max, double w_min, double w_max) : 
        v_min(v_min), v_max(v_max), w_min(w_min), w_max(w_max){}
};

struct Point{
    double x;
    double y;
    Point(double x, double y) : x(x), y(y){}
};


class DWA{
private:
    double dt_;
    double predict_time_;
    double v_max_,v_min_,w_max_,w_min_;
    double a_v_max_,a_w_max_;
    double v_resolution_, w_resolution_;
    double alpha_, beta_, gamma_;
    double radius_;
    double judge_distance_;
private:
    std::vector<State> trajectoryPredict(State state,Control control);
    Window dynamicWinow(const State& state, const std::vector<Point>& obs);
    double getObsDis(const State& state, const std::vector<Point>& obs);
    //获取预测轨迹的最后一个点的状态，求其与goal的各代价
    //double _heading(std::vector<State> trajectory_pre, const Point& goal);
    double _obs(std::vector<State> trajectory_pre, const std::vector<Point>& obs);
    double _vel(std::vector<State> trajectory_pre);
    

public:
    DWA(std::string file_path);
    State kinematicsModel(State state, const Control& control);
    std::tuple<Control, std::vector<State>,std::vector<std::vector<State>>> 
                trajectoryEvaluation(const State& state, const Point& goal, 
                const std::vector<Point>& obs);
};


#endif 