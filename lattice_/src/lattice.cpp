#include "../include/lattice.h"
#include <iostream>
void Path::SL_2_XY(CubicSpline2D& reference_line){
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


void Path::calc_yaw_kappa(){
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

vector<vector<double>> Path::get_path_info(){
    vector<vector<double>> path_info(4);
    path_info[0] = x_;
    path_info[1] = y_;
    path_info[2] = yaw_;
    path_info[3] = kappa_;
    return path_info;
}

//没有考虑到车自身的体积，后续优化
//时间复杂度过高
bool is_collision(const Path& path, const vector<vector<double>>& obs){
    for(size_t i = 0; i < obs[0].size(); i++){
        for(size_t j = 0; j < path.x_.size(); j++){
            if(sqrt(pow(path.x_[j] - obs[0][i],2) + pow(path.y_[j] - obs[1][i],2))-0.5 <= obs[2][i]){
                return 1;
            }
        }
    }
    return 0;
}

lattice_planner::lattice_planner(double l0, double v_l0, double a_l0, 
                                 double s0, double v_s0, double a_s0,
                                 CubicSpline2D& reference_line, vector<double> vehicle_config,
                                 const vector<vector<double>>& obs){
    path_sampling(l0, v_l0, a_l0, s0, v_s0, a_s0, reference_line, vehicle_config, obs);
    cout << "after sampling" << endl;
    extract_optimal_path(obs);

}

void lattice_planner::path_sampling(double l0, double v_l0, double a_l0, 
                                 double s0, double v_s0, double a_s0,
                                 CubicSpline2D& reference_line, vector<double> vehicle_config,
                                 const vector<vector<double>>& obs){
    //constexpr:在编译时就进行初始化，提高算法效率，，在执行中不会改变的常量可用constexpr
    double target_speed = 30/3.6;
    double t_step = 0.2;
    double edge_l = 4;
    double road_sample_step = 2;
    double K_JERK = 0.1;
    double K_TIME = 1.0;
    double K_V_DIFF = 1.0;
    double K_OFFSET = 1.5;
    double K_COLLISION = 500;
    for(double v_expected = target_speed * 0.6; v_expected <= target_speed* 1.4 ; v_expected += target_speed * 0.2){
        //for(double t1 = 4.5; t1 < 5.5; t1 += 0.2){
        for(double t1 = 4.5; t1 < 5.5; t1 += 0.5){

            Path path_pre;
            quarticPolynomial path_s(s0,v_s0,a_s0,v_expected,0,t1);
            for(double t = 0; t < t1; t+=t_step){
                path_pre.t_.push_back(t);
                path_pre.s_.push_back(path_s.calc_s(t));
                path_pre.s_v_.push_back(path_s.calc_v_s(t));
                path_pre.s_a_.push_back(path_s.calc_a_s(t));
                path_pre.s_jerk_.push_back(path_s.calc_jerk_s(t));
            }
            for(double l1 = -edge_l; l1 <= edge_l; l1 += road_sample_step){
                Path path_pre_ = path_pre;
                quinticPolynomial path_l(l0,v_l0,a_l0,l1,0,0,t1);
                for(double t : path_pre_.t_){
                    path_pre_.l_.push_back(path_l.calc_l(t));
                    path_pre_.l_a_.push_back(path_l.calc_a_l(t));
                    path_pre_.l_v_.push_back(path_l.calc_v_l(t));
                    path_pre_.l_jerk_.push_back(path_l.calc_jerk_l(t));
                }

                path_pre_.SL_2_XY(reference_line);
                path_pre_.calc_yaw_kappa();
                if(path_pre_.yaw_.empty()) continue;

                double l_jerk_sum = 0.0;
                double s_jerk_sum = 0.0;
                double s_v_sum = 0.0;
                double v_diff = pow(path_pre_.s_v_.back(), 2);
                for (size_t i = 0; i < path_pre_.l_jerk_.size(); ++i) {
                    l_jerk_sum += abs(path_pre_.l_jerk_[i]);
                    s_jerk_sum += abs(path_pre_.s_jerk_[i]);
                    s_v_sum += abs(path_pre_.s_v_[i]);
                }
                //cout << "jerk_sum = " << K_JERK * (l_jerk_sum + s_jerk_sum) << endl;
                // cout << "v_diff_sum = " << K_V_DIFF * v_diff/100 << endl;
                // cout << " = " << K_TIME * t1 * 2/10<< endl;
                // cout << " = " << K_OFFSET * abs(path_pre_.l_.back()) << endl;
                // cout << " = " << K_COLLISION * is_collision(path_pre_,obs) << endl;
                //path_pre_.cost_ = K_JERK * (l_jerk_sum + s_jerk_sum);
                //path_pre_.cost_ = K_COLLISION * is_collision(path_pre_,obs);
                // path_pre_.cost_ = K_COLLISION * is_collision(path_pre_,obs) + K_JERK * (l_jerk_sum + s_jerk_sum)
                //                 + K_V_DIFF * v_diff / 30 + K_TIME * t1 * 50 + K_OFFSET * abs(path_pre_.l_.back())/2;
                path_pre_.cost_ = K_COLLISION * is_collision(path_pre_,obs) + K_JERK * (l_jerk_sum + s_jerk_sum)
                + K_TIME * t1 * 50 + K_OFFSET * abs(path_pre_.l_.back())/2;
                cout<<"cost = "<<path_pre_.cost_<<endl;
                cout << " time_cost " << K_TIME * t1 * 50 << endl;
                cout << "offset_cost " << K_OFFSET * abs(path_pre_.l_.back())/2  << endl;
                cout << "jerk_cost " << K_JERK * (l_jerk_sum + s_jerk_sum)  << endl;
                cout << "v_diff_cost " << K_V_DIFF * v_diff  << endl;
                paths_.emplace_back(path_pre_);
            }

        }
    }
}

constexpr double MAX_SPEED = 50.0 / 3.6;
constexpr double MAX_ACCEL = 8.0;
constexpr double MAX_CURVATURE = 6.0;

void lattice_planner::extract_optimal_path(const vector<vector<double>>& obs){
    if(paths_.empty()) return;
    std::sort(paths_.begin(),paths_.end());
    for(auto i : paths_){
        for(size_t j = 0; j < i.s_v_.size(); j++){
            if(i.s_v_[j] <= MAX_SPEED && abs(i.s_a_[j]) <= MAX_ACCEL && 
                abs(i.kappa_[j]) <= MAX_CURVATURE){
                optimal_path_ = i;
                break;
            }
        }
    }
}

