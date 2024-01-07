#include "../include/matplotlibcpp.h"
#include "../include/dwa.h"
namespace plt = matplotlibcpp;

//绘制机器人
void plotRobot(double x, double y,double radius){ 
    std::vector<double> points_x;//通过点来构成圆
    std::vector<double> points_y;
    for(double i = 0; i <= 2*PI; i+=0.1){
        points_x.push_back(x+radius*cos(i));
        points_y.push_back(y+radius*sin(i));
    }
    plt::plot(points_x,points_y,"-m");
}

//画箭头
void plotArrow(double x,double y,double yaw,double len = 0.5, double width=0.1){
    plt::arrow(x,y,len*cos(yaw),len*sin(yaw));
    plt::plot(std::vector<double>{x},std::vector<double>{y});
}

int main(){
    //读取配置文件
    std::string filePath = "../config/config.yml";
    YAML::Node config_test = YAML::LoadFile(filePath);
    double dt = config_test["dt"].as<double>();
    double radius = config_test["radius"].as<double>();

    State state(0, 0, 0, 0, 0); //起点
    Point goal(10,14);
    //可以写个结构体表示障碍物，顺便把半径加上，getObsDis也要相应修改，复用plotRobot绘制
    std::vector<Point> obs = {{-1, -1}, {0, 2}, {4, 2}, {5, 4}, {5, 5}, {5, 6}, {5, 9}, {8, 9}, 
                              {7, 9}, {8, 10}, {9, 11}, {12, 13}, {12,  12}, {15, 15}, {13, 13},
                              {5, 10}, {3, 5}, {4, 4}, {4, 11}, {15, 5}, {8, 13},{5, 13},
                              {5, 5}, {2, 8}};
    // std::vector<Point> obs = {{0, 2}, {4, 2},  {8, 9}, 
    //                         {7, 9}, {8, 10}, {12, 13}, {12,  12}, {15, 15}, {13, 13},
    //                          {8, 13},{5, 13},
    //                         {5, 5}, {2, 8}};

    std::vector<State> trajectory; //最优轨迹
    DWA dwa(filePath);
    std::vector<double>x_,y_,predict_x,predict_y;
    while(true){
        std::tuple<Control, std::vector<State>,std::vector<std::vector<State>>> 
            ans = dwa.trajectoryEvaluation(state,goal,obs);
        state = dwa.kinematicsModel(state,std::get<0>(ans));
        trajectory.push_back(state);

        x_.push_back(state.x);
        y_.push_back(state.y);

        //画图
        plt::clf();
        plt::plot(std::vector<double>{state.x},std::vector<double>{state.y},"xr");
        plt::plot(std::vector<double>{goal.x},std::vector<double>{goal.y},"rs");//目标
        for(Point i : obs){//障碍物
            plt::plot(std::vector<double>{i.x},std::vector<double>{i.y},"ok");
        }
        plotArrow(state.x,state.y,state.yaw);
        plotRobot(state.x, state.y, radius);
        //画出最优轨迹
        for(std::vector<State> i : std::get<2>(ans)){
            for(State s : i){
                predict_x.push_back(s.x);
                predict_y.push_back(s.y);
            }
            plt::plot(predict_x,predict_y,"-y");
            predict_x = {};
            predict_y = {};
        }
        //画出所有的采样轨迹
        for(State s : std::get<1>(ans)){
            predict_x.push_back(s.x);
            predict_y.push_back(s.y);
        }
        plt::plot(predict_x,predict_y,"-g");
        predict_x = {};
        predict_y = {};
        //画机器人轨迹
        plt::plot(x_,y_,"r");
        plt::grid(true);
        plt::pause(0.01);
        //终止条件，碰到目标点即为到达
        double dis_to_goal = hypot(state.x - goal.x, state.y - goal.y);
        if(dis_to_goal<=radius)break;
    }

    //// save figure
    const char* filename = "./dwa_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();


    return 0;
}