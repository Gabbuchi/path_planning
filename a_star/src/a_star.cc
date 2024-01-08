#include "../include/a_star.h"
#include "../include/plot_tool.h"

//Node
Node::Node(Point point) { point_ = point; }
void Node::set_location(Point point) { point_ = point; }
Point Node::get_location() { return point_; }
void Node::set_father(Node* father) { father_ = father; }
Node* Node::get_father() { return father_; }
std::vector<Node*> Node::get_neighbors() { return neighbors_; }
void Node::set_neighbors(std::vector<Node*> neighbors) { neighbors_ = neighbors; }
void Node::set_obs() {is_obs_ = 1;}
bool Node::is_obs() {return is_obs_;}
void Node::set_path_length_cost(){
    if(father_ == nullptr){
        path_length_cost = 0;
        return;
    }
    path_length_cost = father_->get_path_length_cost() + 1;
}
double Node::get_path_length_cost(){return path_length_cost;}



//heap
bool minHeap::isVisited(Node* node){ return indexMap_.count(node); }
bool minHeap::isInHeap(Node* node){return isVisited(node) && indexMap_[node] != -1;}

void minHeap::swap(int index1, int index2){
    Node* temp = nodes_[index1];
    nodes_[index1] = nodes_[index2];
    nodes_[index2] = temp;
    indexMap_[nodes_[index1]] = index2;
    indexMap_[nodes_[index2]] = index1;
}

void minHeap::heapify(int index){
    int size = nodes_.size();
    int left = index*2+1;
    while(left < size){
        int smallest = left + 1 < size && 
                        distanceMap_[nodes_[left + 1]] < distanceMap_[nodes_[left]]
                        ?left + 1 : left;
        smallest = distanceMap_[nodes_[smallest]] < distanceMap_[nodes_[index]] ? smallest : index;
        if(smallest == index) break;
        swap(smallest,index);
        index = smallest;
        left = index*2 + 1;
    }
}

bool minHeap::empty(){return nodes_.empty();}


void minHeap::insert(Node* last_node,Node* node,double cost){
    nodes_.push_back(node);
    int index = nodes_.size() - 1;
    indexMap_[node] = index;
    distanceMap_[node] = cost;
    //调整堆
    while(index > 0 && distanceMap_[nodes_[index]] <= distanceMap_[nodes_[(index-1)/2]]){
        swap(index,(index-1)/2);
        index = (index-1)/2;
    }
    node->set_father(last_node);
    plotTools::fillOneGrid(node->get_location().x,node->get_location().y,"green","0.6");
    plt::pause(0.01);//暂停方便观察搜索过程
}

nodeRecord minHeap::pop(){
    Node* heap_top_node = nodes_[0];
    double cost = distanceMap_[heap_top_node];
    nodeRecord myRecord(heap_top_node,cost);
    swap(0,nodes_.size()-1);
    indexMap_[heap_top_node] = -1;
    distanceMap_.erase(heap_top_node);
    nodes_.pop_back();
    heapify(0);
    return myRecord;
}


//Graph
Graph::Graph(int rows, int cols, std::vector<Obstacle> obstacles) : dimension_x_(cols), dimension_y_(rows){
    Init();
    set_obs(obstacles);
}
void Graph::free_nodes() {
    for (auto& row : nodes_) {
        for (auto node : row) {
            delete node;
        }
    }
    std::cout<<"nodes have been free"<< std::endl;
}
int Graph::get_dimension_x() {return dimension_x_;}
int Graph::get_dimension_y() {return dimension_y_;}
std::vector<std::vector<Node*>> Graph::get_nodes(){return nodes_;}

void Graph::Init(){
    for(int i = 0; i < dimension_x_; i++){
        std::vector<Node*> row;
        for(int j = 0; j < dimension_y_; j++){
            row.push_back(new Node(Point(i,j))); //set_location
        }
        nodes_.push_back(row);
    }
}

void Graph::set_obs(const std::vector<Obstacle>& obstacles){
    for(Obstacle obs : obstacles){
        for(int i = obs.bottom_left_x; i < obs.top_right_x;i++){
            for(int j = obs.bottom_left_y; j < obs.top_right_y; j++){
                nodes_[i][j]->set_obs();
            }
        }
        plotTools::fillGrid({double(obs.bottom_left_x),double(obs.bottom_left_y)},
                            {double(obs.top_right_x),double(obs.top_right_y)},"black","0.9");
    }
}




