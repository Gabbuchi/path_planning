#ifndef RRT_H_
#define RRT_H_

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

struct Point {
  Point(double x, double y) : x(x), y(y) {}
  double x = 0.0;
  double y = 0.0;
};

class Node {
 public:
  Node(double x, double y) : point_(x, y), parent_(nullptr) {}  //构造函数
  const Point& point() const { return point_; } //返回节点坐标
  void set_parent(Node* parent) { parent_ = parent; } //设置父节点
  Node* parent() { return parent_; }  //返回父节点

 private:
  Point point_;  //当前Node的坐标
  Node* parent_ = nullptr; //父节点指针
  //double cost_ = 0.0;  //代价
};

class RRT {
 public:
  RRT(Node* start_node, Node* goal_node,  //起始、终止点
      const std::vector<std::vector<double>>& obstacle_list,
      double step_size = 1.0, int goal_sample_rate = 5)  // 障碍物，步长（节点之间的距离），目标采样率（有多少概率直接采样到目标点）
      : start_node_(start_node),
        goal_node_(goal_node),
        obstacle_list_(obstacle_list),
        step_size_(step_size),
        goal_sample_rate_(goal_sample_rate),
        goal_gen_(goal_rd_()),
        goal_dis_(std::uniform_int_distribution<int>(0, 100)),
        area_gen_(area_rd_()),
        area_dis_(std::uniform_real_distribution<double>(0, 15)) {}
  Node* GetNearestNode(const std::vector<double>& random_position);
  bool CollisionCheck(Node* nereastNode, Node* newNode);  //是否碰撞
  std::vector<Node*> Planning() ;  //路径

 private:
  Node* start_node_;
  Node* goal_node_;
  std::vector<std::vector<double>> obstacle_list_;
  std::vector<Node*> node_list_;
  double step_size_;

  int goal_sample_rate_;

  std::random_device goal_rd_;
  std::mt19937 goal_gen_;
  std::uniform_int_distribution<int> goal_dis_;

  std::random_device area_rd_;
  std::mt19937 area_gen_;
  std::uniform_real_distribution<double> area_dis_;
};

#endif
