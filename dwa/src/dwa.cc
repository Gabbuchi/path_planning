#include "../include/dwa.h"

//读取config文件并初始化成员变量
DWA::DWA(std::string filePath){ 
    YAML::Node config = YAML::LoadFile(filePath);
    dt_ = config["dt"].as<double>();
    v_min_ = config["vMin"].as<double>();
    v_max_ = config["vMax"].as<double>();
    w_min_ = config["wMin"].as<double>();
    w_max_ = config["wMax"].as<double>();;
    predict_time_ = config["predictTime"].as<double>();
    a_v_max_ = config["aVmax"].as<double>();
    a_w_max_ = config["aWmax"].as<double>();
    v_resolution_ = config["vSample"].as<double>();
    w_resolution_ = config["wSample"].as<double>();;
    alpha_ = config["alpha"].as<double>();
    beta_ = config["beta"].as<double>();
    gamma_ = config["gamma"].as<double>();
    radius_ = config["radius"].as<double>();
    judge_distance_ = config["judgeDistance"].as<double>();
}

//运动学模型（二维平面）
State DWA::kinematicsModel(
    State state,
    const Control& control){

    state.x += control.v * cos(state.yaw) * dt_;
    state.y += control.v * sin(state.yaw) * dt_;
    state.yaw += control.w * dt_;
    state.v = control.v;
    state.w = control.w;

    return state;
}

//轨迹预测
std::vector<State> DWA::trajectoryPredict(State state,Control control){
    std::vector<State> trajectory;
    trajectory.push_back(state);
    for(double count = 0; count <= predict_time_; count += dt_){
        state = kinematicsModel(state,control);
        trajectory.push_back(state);
    }
    return trajectory;
}

//获取障碍物距离
double DWA::getObsDis(const State& state, const std::vector<Point>& obs){
    double min_dis = std::numeric_limits<double>::infinity();
    for(const Point& i : obs){
        double dis = hypot(state.x - i.x, state.y - i.y);//计算欧氏距离
        min_dis = dis < min_dis ? dis : min_dis;
    }
    return min_dis;
}

//生成动态窗口
Window DWA::dynamicWinow(const State& state, const std::vector<Point>& obs){
    Window obs_limit,accLimit,vel_limit, my_window;
    accLimit.v_max = state.v + a_v_max_ * dt_;
    accLimit.v_min = state.v - a_v_max_ * dt_;
    accLimit.w_max = state.w + a_w_max_ * dt_;
    accLimit.w_min = state.w - a_w_max_ * dt_;

    obs_limit.v_max = sqrt(2*getObsDis(state,obs)*a_v_max_);
    obs_limit.v_min = v_min_;
    obs_limit.w_max = w_max_;
    obs_limit.w_min = w_min_;

    // vel_limit.v_max = v_max_;
    // vel_limit.v_min = v_min_;
    // vel_limit.w_max = w_max_;
    // vel_limit.w_min = w_min_;

    my_window.v_max = std::min(std::min(accLimit.v_max, obs_limit.v_max), v_max_);
    my_window.v_min = std::max(std::max(accLimit.v_min, obs_limit.v_min), v_min_);
    my_window.w_max = std::min(std::min(accLimit.w_max, obs_limit.w_max), w_max_);
    my_window.w_min = std::max(std::max(accLimit.w_min, obs_limit.w_min), w_min_);

    return my_window;
}

// double DWA::_heading(std::vector<State> trajectory_pre,const Point& goal){ //原作者的方法，我复现的时候有问题
//     State state_last = trajectory_pre.back();
//     double angle_with_goal = atan2((goal.y - state_last.y),(goal.x - state_last.x)); //末点与目标连成的向量的方向角
//     // std::cout << "angle with goal:" << angle_with_goal << std::endl;
//     double error_angle = angle_with_goal - state_last.yaw;
//     //std::cout << "error angle:" << error_angle << std::endl;
//     return PI - abs(error_angle);
// }

double DWA::_obs(std::vector<State> trajectory_pre, const std::vector<Point>& obs){
    double min_dis = std::numeric_limits<double>::infinity();
    for(Point i : obs){
        for(const State& j : trajectory_pre){
            double dis = hypot(j.x - i.x, j.y - i.y);//计算欧氏距离的
            min_dis = dis < min_dis ? dis : min_dis;
        }
    }
    if(min_dis < radius_ + 0.2)return min_dis;
    else return judge_distance_;
}

//用trajectory终点的预测速度作为速度得分
double DWA::_vel(std::vector<State> trajectory_pre){ 
    return trajectory_pre.back().v;
} 


std::tuple<Control, std::vector<State>,std::vector<std::vector<State>>> 
    DWA::trajectoryEvaluation(const State& state, const Point& goal, 
    const std::vector<Point>& obs){
    
    double score_best = -std::numeric_limits<double>::infinity(); //得分
    std::vector<State> trajectory_optimal;
    Control control_optimal;

    Window dynamic_window = dynamicWinow(state,obs);
    std::cout << "vel_limit:" << dynamic_window.v_min << " " << dynamic_window.v_max << std::endl; 
    std::cout << "w_limit:" << dynamic_window.w_min << " " << dynamic_window.w_max << std::endl; 

    std::vector<std::vector<State>> trajectories;

    double v = dynamic_window.v_min;
    double w = dynamic_window.w_min;
    for(double v = dynamic_window.v_min; v <= dynamic_window.v_max; v += v_resolution_){
        for(double w = dynamic_window.w_min; w <= dynamic_window.w_max; w += w_resolution_){
            Control control = {v,w};
            std::vector<State> trajectory = trajectoryPredict(state,control);//轨迹预测
            trajectories.push_back(trajectory);
            //double heading_score = alpha_ * _heading(trajectory,goal) / sum_heading;
            //std::cout << "heading score:" << heading_score << std::endl;
            double obs_score = beta_ * _obs(trajectory,obs);
            double vel_score = gamma_ * _vel(trajectory);
            double score = obs_score + vel_score + 1/hypot(trajectory.back().x - goal.x,trajectory.back().y - goal.y);
            if(score >= score_best){
                score_best = score;
                trajectory_optimal = trajectory;
                control_optimal = control;
            }
        }
    }
    return {control_optimal,trajectory_optimal,trajectories};

}