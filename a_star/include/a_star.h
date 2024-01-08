#ifndef A_STAR_CJL_H
#define A_STAR_CJL_H

#include <iostream>
#include <vector>
#include <unordered_map>

struct Point{
    int x;
    int y;
    Point(int x, int y) : x(x), y(y){}
    Point() : x(0), y(0){}
};


class Node{
public:
    Node(Point point);
    void set_location(Point point);
    Point get_location();
    void set_father(Node* father);
    Node* get_father();
    std::vector<Node*> get_neighbors();
    void set_neighbors(std::vector<Node*> neighbors);
    void set_obs();
    bool is_obs();
    void set_path_length_cost();
    double get_path_length_cost();

private:
    Point point_;
    Node* father_ = nullptr;
    std::vector<Node*> neighbors_;
    bool is_obs_ = 0;
    double path_length_cost;
};

//heap
struct nodeRecord{
    Node* node;
    int distance;
    nodeRecord(Node* node,int distance) : node(node),distance(distance){}
};

class minHeap{
public:
    bool isVisited(Node* node);
private:
    bool isInHeap(Node* node);
    void swap(int index1, int index2);
    void heapify(int index);
public:
    bool empty();
    void insert(Node* last_node,Node* node,double cost);
    nodeRecord pop();
private:
    std::vector<Node*> nodes_;
    std::unordered_map<Node*,int> indexMap_;//索引表，用来记录node的位置
    std::unordered_map<Node*,double> distanceMap_;
};

struct Obstacle{
    int bottom_left_x;
    int bottom_left_y;
    int top_right_x;
    int top_right_y;
    Obstacle(int bottom_left_x, int bottom_left_y, int top_right_x, int top_right_y) : 
                bottom_left_x(bottom_left_x),bottom_left_y(bottom_left_y),
                top_right_x(top_right_x),top_right_y(top_right_y){}
};

struct Graph{
public:
    Graph(int rows, int cols, std::vector<Obstacle> obstacles);
    void free_nodes();
    int get_dimension_x();
    int get_dimension_y();
    std::vector<std::vector<Node*>> get_nodes();
private:
    void Init();
    void set_obs(const std::vector<Obstacle>& obstacles);
private:
    std::vector<std::vector<Node*>> nodes_;
    int dimension_x_;
    int dimension_y_;
};


#endif 
