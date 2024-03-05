#include "../include/rrt.h"
#include "../include/cubic_spline.h"
#include "../include/matplotlibcpp.h"
#include "../include/polynomial_curve.h"
#include "../include/lattice.h"
#include <iostream>
using namespace std;
namespace plt = matplotlibcpp;

//未考虑车的体积
//由于没有预测模块，动态避障效果不好
//轨迹评价函数没写好，需要归一化
//碰撞检测模块没有考虑车体积

int main(int argc, char* argv[]) {
    // (x, y, r)
    std::vector<std::vector<double>> obstacle_list{{70, 50, 5}, {60, 30, 15}, {40, 60, 10}, {50, 80, 10},
                                        {30, 100, 10}, {90, 40, 10}, {100, 60, 10}, {110, 70, 10}};
    
    // vector<vector<double>> obs_list_global;
    // for(auto i : obstacle_list){
    //     obs_list_global[0].push_back(i[0]);
    //     obs_list_global[1].push_back(i[1]);
    //     obs_list_global[2].push_back(i[2]);
    // }

    Node* start_node = new Node(1.0, 1.0);
    Node* goal_node = new Node(140, 140);

    RRT rrt(start_node, goal_node, obstacle_list, 5, 5); 
    std::vector<std::vector<double>> path_raw(2);
    std::vector<std::vector<double>> path_raw_(2);
    std::vector<Node*> path = rrt.Planning();

    for(auto i : path){
        path_raw_[0].push_back(i->point().x);
        path_raw_[1].push_back(i->point().y);
    }  
    std::reverse(path_raw_[0].begin(),path_raw_[0].end());
    std::reverse(path_raw_[1].begin(),path_raw_[1].end());
    
    //对最后一个点进行采样
    for(int i = 0; i < path_raw_[0].size(); i += 7){
        path_raw[0].push_back(path_raw_[0][i]);
        path_raw[1].push_back(path_raw_[1][i]);
    }
    if(path_raw_[0].size() % 7 != 0){
        path_raw[0].push_back(path_raw_[0][path_raw_[0].size()-1]);
        path_raw[1].push_back(path_raw_[1][path_raw_[0].size()-1]);
    }

    //此方式拟合有可能产生碰撞，后续局部路径规划也可解决碰撞
    CubicSpline2D my_spline2D(path_raw[0],path_raw[1]);
    vector<vector<double>> sp_path_ = my_spline2D.calc_path_with_xy_yaw_curv(0.1);

    // vector<vector<double>> obs = {{65,100,20,43},{90,75,57,85},{2,2,3,3}};
    vector<double> vc;
    double l0 = 0.0;    // current lateral position [m] 
    double l0_v = 0.0;  // current lateral speed [m/s]
    double l0_a = 0.0;  // current lateral acceleration [m/s]
    double s0 = 0.0;    // current course position
    double s0_v = 30.0 / 3.6;   // current speed [m/s]
    double s0_a = 0.0;
    plt::figure_size(1200,1200);
    plt::figure(2);
    plt::figure_size(1200,1200);
    plt::figure(3);
    vector<vector<double>> path_final(2);
    double obs_longitude_x = 15;
    bool move_right = true;
    double obs_step_x = 0;
    while(1){

        if(move_right){
            obs_step_x += 0.05;
            if(obs_step_x > 3) move_right = false;
        }else{
            obs_step_x -= 0.05;
            if(obs_step_x < 0) move_right = true;
        }
        // vector<vector<double>> obs_local = {{15,15,10,40,15,112,70,120},
        //                                 {20,30,40,10,60,100,80,120},{2,4,2,3,2,3,2,3}};
        vector<vector<double>> obs_local = {{15,15,10,40,15,112,70,120,45,45,40,70,110,80,90},
                                        {20,30,40,10,60,100,80,120,50,60,70,40,115,105,20},{2,2,2,3,2,3,2,3,2,4,2,3,3,2,2}};
        for(size_t i = 0; i < obs_local[0].size(); i++){
            obs_local[0][i] += obs_step_x;
        }
        vector<vector<double>> obs = obs_local;
        for(auto i : obstacle_list){
            obs[0].push_back(i[0]);
            obs[1].push_back(i[1]);
            obs[2].push_back(i[2]);
        }
        if(abs(s0 - my_spline2D.get_s().back()) < 1) break;
        lattice_planner lattice(l0, l0_v, l0_a, s0, s0_v, s0_a, my_spline2D, vc, obs);
        Path path_next = lattice.get_optimal_path();
        l0 = path_next.l_[1];
        l0_v = path_next.l_v_[1];
        l0_a = path_next.l_a_[1];
        s0 = path_next.s_[1];
        s0_v = path_next.s_v_[1];
        s0_a = path_next.s_a_[1];
        path_final[0].push_back(path_next.x_[0]);
        path_final[1].push_back(path_next.y_[0]);
        vector<Path> paths = lattice.get_paths();
        if(path_next.x_.empty()) break;
        vector<vector<vector<double>>> path_sample_;
        for(auto _path : paths){
            path_sample_.push_back({_path.x_,_path.y_});
        }



        double center_x = path_next.x_[0] + 10; 
        double center_y = path_next.y_[0] + 10; 
        double range_x = 50.0; // x 轴的显示范围
        double range_y = 50.0; // y 轴的显示范围
        vector<double> state_x = {path_next.x_[0]};
        vector<double> state_y = {path_next.y_[0]};

        plt::figure(2);
        plt::cla();
        for(auto i = 0; i < path_sample_.size(); i++){
            plt::named_plot("",path_sample_[i][0], path_sample_[i][1],"b");
        }
        for(size_t i = 0; i < obs[0].size(); i++){
            plotRobot(obs[0][i],obs[1][i],obs[2][i]);
        }
        // 设置 x 轴和 y 轴的显示范围，使得中心点始终在视图中心
        plt::xlim(center_x - range_x / 2, center_x + range_x / 2);
        plt::ylim(center_y - range_y / 2, center_y + range_y / 2);
        plt::named_plot("path_after_optimize", sp_path_[0], sp_path_[1], "g--");
        plt::named_plot("path_lattice", path_next.x_, path_next.y_, "r");
        plt::named_plot("current position", state_x, state_y, "xb"); 
        plt::grid(true);
        plt::legend();
        plt::title("lattice");


        plt::figure(3);
        plt::cla();
        for(auto i = 0; i < path_sample_.size(); i++){
            plt::named_plot("",path_sample_[i][0], path_sample_[i][1],"b");
        }
        for(size_t i = 0; i < obs[0].size(); i++){
            plotRobot(obs[0][i],obs[1][i],obs[2][i]);
        }
        plt::named_plot("path_after_optimize", sp_path_[0], sp_path_[1], "g--");
        plt::named_plot("path_lattice", path_next.x_, path_next.y_, "r");
        plt::named_plot("path final", path_final[0], path_final[1], "m"); 
        plt::named_plot("current position", state_x, state_y, "xb"); 
        plt::grid(true);
        plt::legend();
        plt::title("lattice");
        plt::pause(0.0001);
    }  
    plt::show();
  return 0;
}