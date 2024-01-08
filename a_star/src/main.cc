#include "../include/matplotlibcpp.h"
#include "../include/a_star.h"
#include "../include/plot_tool.h"
namespace plt = matplotlibcpp;

double heuristicCost(Node* cur, Node* goal){
    return hypot(goal->get_location().x - cur->get_location().x, 
                 goal->get_location().y - cur->get_location().y);
}

bool isCoordinateValid(Graph graph, int x, int y){
    return x >=0 && x < graph.get_dimension_x() && y>=0 && y < graph.get_dimension_y();
}

void setNeighbors(Node* node,Graph graph,minHeap heap){
    int x = node->get_location().x;
    int y = node->get_location().y;
    std::vector<Node*> neis;
    if(isCoordinateValid(graph,x-1,y-1) && !graph.get_nodes()[x-1][y-1]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x-1][y-1])){
            neis.push_back(graph.get_nodes()[x-1][y-1]);
        } 
    }
    if(isCoordinateValid(graph,x,y-1) && !graph.get_nodes()[x][y-1]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x][y-1])){
            neis.push_back(graph.get_nodes()[x][y-1]);
        }
    }
    if(isCoordinateValid(graph,x+1,y-1) && !graph.get_nodes()[x+1][y-1]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x+1][y-1])){
            neis.push_back(graph.get_nodes()[x+1][y-1]);            
        }        
    }
    if(isCoordinateValid(graph,x+1,y) && !graph.get_nodes()[x+1][y]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x+1][y])){      
            neis.push_back(graph.get_nodes()[x+1][y]);     
        }
    }
    if(isCoordinateValid(graph,x+1,y+1) && !graph.get_nodes()[x+1][y+1]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x+1][y+1])){  
            neis.push_back(graph.get_nodes()[x+1][y+1]);          
        }

    }
    if(isCoordinateValid(graph,x,y+1)){
        if(!heap.isVisited(graph.get_nodes()[x][y+1]) && !graph.get_nodes()[x][y+1]->is_obs()){      
            neis.push_back(graph.get_nodes()[x][y+1]);      
        }
    }
    if(isCoordinateValid(graph,x-1,y+1) && !graph.get_nodes()[x-1][y+1]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x-1][y+1])){ 
            neis.push_back(graph.get_nodes()[x-1][y+1]);             
        }
    }
    if(isCoordinateValid(graph,x-1,y) && !graph.get_nodes()[x-1][y]->is_obs()){
        if(!heap.isVisited(graph.get_nodes()[x-1][y])){     
            neis.push_back(graph.get_nodes()[x-1][y]);
        }
    }
    node->set_neighbors(neis);
}

Node* aStar(Node* start, Node* goal,Graph graph){
    minHeap my_priority_queue;
    my_priority_queue.insert(nullptr,start,heuristicCost(start,goal));
    while(!my_priority_queue.empty()){
        nodeRecord cur = my_priority_queue.pop();
        Node* node_cur = cur.node;
        if(node_cur == goal){
            std::cout << "path have been found!" << std::endl;
            return node_cur;
        }       
        double cost_cur = cur.distance;
        setNeighbors(node_cur,graph,my_priority_queue);
        for(Node* neighbor : node_cur->get_neighbors()){
            neighbor->set_path_length_cost();
            my_priority_queue.insert(node_cur,neighbor,node_cur->get_path_length_cost() + heuristicCost(neighbor,goal));
        }
    }
    return nullptr;
}

int main(){

    plotTools::plotBackground(20,20);
    Obstacle obstacle1(3,4,6,6);
    Obstacle obstacle2(6,7,8,8);
    //2024.1.6
    //Obstacle obstacle3(3,9,10,12);
    Obstacle obstacle3(11,10,15,13);
    //Obstacle obstacle3(3,9,14,12);
    //Obstacle obstacle4(14,13,17,15);
    std::vector<Obstacle> obs{obstacle1,obstacle2,obstacle3};
    Graph myGraph(20,20,obs);
    Node* start = myGraph.get_nodes()[0][0];
    plotTools::fillOneGrid(start->get_location().x,start->get_location().y,"blue","0.9");
    Node* goal = myGraph.get_nodes()[12][18];
    //Node* goal = myGraph.get_nodes()[17][1];
    //Node* goal = myGraph.get_nodes()[2][17];
    plotTools::fillOneGrid(goal->get_location().x,goal->get_location().y,"blue","0.9");
    Node* ans = aStar(start,goal,myGraph);
    std::vector<Node*> myAns;
    if(ans == nullptr){
        std::cout << "no ans" << std::endl;
    }else{
        while(ans->get_father() != nullptr){
            plotTools::fillOneGrid(ans->get_location().x,ans->get_location().y,"red","0.7");
            plt::pause(0.01);       
            myAns.push_back(ans);
            ans = ans->get_father();
        }
        for(auto i : myAns){
            plotTools::fillOneGrid(i->get_location().x,i->get_location().y,"red","0.7");
            plotTools::fillOneGrid(goal->get_location().x,goal->get_location().y,"blue","0.9");
        }
    }
    
    myGraph.free_nodes();
    plt::axis("equal");
    plt::show();

    return 0;
}
