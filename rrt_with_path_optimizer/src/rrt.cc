#include "../include/rrt.h"
#include "../include/cubic_spline.h"
#include "../include/matplotlibcpp.h"


namespace plt = matplotlibcpp;

void plotRobot(double x, double y,double radius){ 
    std::vector<double> points_x;//通过点来构成圆
    std::vector<double> points_y;
    for(double i = 0; i <= 2*3.14159265 + 0.1; i+=0.1){
        points_x.push_back(x+radius*cos(i));
        points_y.push_back(y+radius*sin(i));
    }
    plt::plot(points_x,points_y,"-m");
}

//遍历找到离随机点最近的节点
Node* RRT::GetNearestNode(const std::vector<double>& random_position) {
  int min_id = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < node_list_.size(); i++) {
    double square_distance =
        std::pow(node_list_[i]->point().x - random_position[0], 2) +
        std::pow(node_list_[i]->point().y - random_position[1], 2);
    if (square_distance < min_distance) {
      min_distance = square_distance;
      min_id = i;
    }
  }

  return node_list_[min_id];
}

//碰撞检测
bool RRT::CollisionCheck(Node* father, Node* newNode) {
  for (auto item : obstacle_list_) {
    if (std::sqrt(std::pow((item[0] - newNode->point().x), 2) +
                  std::pow((item[1] - newNode->point().y), 2)) <= item[2] || 
        std::sqrt(std::pow((item[0] - father->point().x), 2) +
                  std::pow((item[1] - father->point().y), 2)) <= item[2]) //与圆心距离小于半径则在障碍内
      return false;

    double dx = newNode->point().x - father->point().x;
    double dy = newNode->point().y - father->point().y;
    double a = std::pow(dx, 2) + std::pow(dy, 2);
    double b = 2 * (dx * (newNode->point().x - item[0]) + dy * (newNode->point().y - item[1]));
    double c = std::pow(newNode->point().x - item[0], 2) + std::pow(newNode->point().y - item[1], 2) - std::pow(item[2], 2);

    double discriminant = b * b - 4 * a * c;
    double t1 = (-b + std::sqrt(discriminant)) / (2 * a);
    double t2 = (-b - std::sqrt(discriminant)) / (2 * a);
    if (discriminant >= 0 || (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) return false;

  }
  return true;
}



std::vector<Node*> RRT::Planning() {
  
  plt::figure_size(1200,1200);

  plt::clf();
  plt::plot({start_node_->point().x},{start_node_->point().y},"xr");
  plt::plot({goal_node_->point().x},{goal_node_->point().y},"rs");//目标
  for(auto i : obstacle_list_){//障碍物
      plotRobot(i[0],i[1],i[2]);
  }


  node_list_.push_back(start_node_); //node_list_用于记录所有的树的节 点
  while (1) {
    std::vector<double> random_position; //存储生成的随机位置

    if (goal_dis_(goal_gen_) > goal_sample_rate_) { //如果生成的随机数大于目标采样概率，则生成随机点
      double randX = area_dis_(goal_gen_); //生成随机点X,Y
      double randY = area_dis_(goal_gen_);
      random_position.push_back(randX);
      random_position.push_back(randY);
    } else {   //否则目标点则为随机点
      random_position.push_back(goal_node_->point().x);
      random_position.push_back(goal_node_->point().y);
    }

    Node* nearestNode = GetNearestNode(random_position); //获取最近节点

    double theta = atan2(random_position[1] - nearestNode->point().y,
                         random_position[0] - nearestNode->point().x);  //随机点与最近点的夹角

    Node* newNode = new Node(nearestNode->point().x + step_size_ * cos(theta),
                             nearestNode->point().y + step_size_ * sin(theta)); //新节点的坐标（先生成随机点，然后取长度为步长，夹角为随机点夹角创建新节点）
    
    newNode->set_parent(nearestNode); //设置父节点

    if (!CollisionCheck(nearestNode,newNode)) continue;  
    
    node_list_.push_back(newNode); //新节点插入node_list_

    plt::plot({newNode->point().x,nearestNode->point().x},{newNode->point().y,nearestNode->point().y},"g");

    if (sqrt(pow(newNode->point().x - goal_node_->point().x, 2) +
             pow(newNode->point().y - goal_node_->point().y, 2)) <=
        step_size_) {
      std::cout << "path has been found!" << std::endl;
      break;
    }
  }

  std::vector<Node*> path;
  path.push_back(goal_node_);
  Node* tmp_node = node_list_.back();


  while (tmp_node->parent() != nullptr) {
    plt::plot({tmp_node->point().x,tmp_node->parent()->point().x},{tmp_node->point().y,tmp_node->parent()->point().y},"r");
    path.push_back(tmp_node);
    tmp_node = tmp_node->parent();
  }

  path.push_back(start_node_);
  return path;
}

int main(int argc, char* argv[]) {
  // (x, y, r)
  std::vector<std::vector<double>> obstacle_list{{7, 5, 0.5}, {6, 3, 1.5}, {4, 6, 1}, {5, 8, 1},
                                     {3, 10, 1}, {9, 4, 1}, {10, 6, 1}, {11, 7, 1}};

  Node* start_node = new Node(1.0, 1.0);
  Node* goal_node = new Node(14.0, 14.0);

  RRT rrt(start_node, goal_node, obstacle_list, 0.5, 5); 
  std::vector<std::vector<double>> path_raw(2);
  std::vector<std::vector<double>> path_raw_(2);
  std::vector<Node*> path = rrt.Planning();

  for(auto i : path){
    path_raw_[0].push_back(i->point().x);
    path_raw_[1].push_back(i->point().y);
  }  

  for(int i = 0; i < path_raw_[0].size(); i += 7){
    path_raw[0].push_back(path_raw_[0][i]);
    path_raw[1].push_back(path_raw_[1][i]);
  }

  //int count = 0;
  //std::copy_if(original.begin(), original.end(), std::back_inserter(sampled),
  //               [&count](int) { return count++ % 3 == 0; });

  //此方式拟合有可能产生碰撞，后续局部路径规划也可解决碰撞
  CubicSpline2D my_spline2D(path_raw[0],path_raw[1]);
  vector<vector<double>> sp_path_ = my_spline2D.calc_path_with_xy_yaw_curv(0.1);

  //plt::named_plot("raw_data", path_raw[0], path_raw[1], "r");
  plt::named_plot("Cubic spline2D interpolation", sp_path_[0], sp_path_[1], "b");
  plt::figure_size(1200,1200);
  plt::named_plot("Cubic spline2D interpolation", sp_path_[0], sp_path_[1], "r");
  plt::grid(true);
  plt::legend();
  plt::title("Cubic Spline");
  plt::show();


  return 0;
}
