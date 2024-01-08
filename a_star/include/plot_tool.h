#ifndef PLOT_TOOL_CJL_H
#define PLOT_TOOL_CJL_H
#include "../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

class plotTools{
public:

    //绘制空格作为背景
    static void plotBackground(double rows, double cols){
        for(double i = 0; i <= rows; i++){
            plt::plot({0,cols},{i,i},"k-");
        }
        for(double i = 0; i <= cols; i++){
            plt::plot({i,i},{0,rows},"k-");
        }
    }

    //绘制(填充)大面积格子
    static void fillGrid(std::vector<double> bottom_left, std::vector<double> top_right,
                         std::string color,std::string alpha){
        std::vector<double> x{bottom_left[0],top_right[0]};
        std::vector<double> y1{bottom_left[1],bottom_left[1]};
        std::vector<double> y2{top_right[1],top_right[1]};
        std::map<std::string, std::string> fill_params;
        fill_params["color"] = color;
        fill_params["alpha"] = alpha;
        plt::fill_between(x,y1,y2,fill_params);
    }

    //绘制（填充）单个格子
    static void fillOneGrid(double x_low_left,double y_low_left,std::string color,std::string alpha){
        std::vector<double> x{x_low_left,x_low_left + 1};
        std::vector<double> y1{y_low_left,y_low_left};
        std::vector<double> y2{y_low_left + 1,y_low_left + 1};
        std::map<std::string, std::string> fill_params;
        fill_params["color"] = color;
        fill_params["alpha"] = alpha;
        plt::fill_between(x,y1,y2,fill_params);
    }
};

#endif 