#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>
#include <set>

struct Edge;

struct Node
{
    int val;
    Node* father;
    std::vector<Node*> nexts;
    std::vector<Edge*> edges;
    Node(int val):val(val),nexts(),edges(){}
};

struct Edge{
    int weight;
    Node* from;
    Node* to;
    Edge(int weight,Node* from,Node* to) : weight(weight),from(from),to(to){}
    Edge():weight(0),from(nullptr),to(nullptr) {}
};

struct Graph{
    std::unordered_map<int,Node*> nodes;
    std::unordered_set<Edge*> edges;
    ~Graph() {
        for (auto& entry : nodes) {
            for (Edge* edge : entry.second->edges) {
                delete edge;
            }
            delete entry.second;
        }
        nodes.clear();
    }
};

class nodeRecord{
    public:
    Node* node;
    int distance;
    nodeRecord(Node* node,int distance) : node(node),distance(distance){}
};

class minHeap{
    //堆
    std::vector<Node*> nodes;
    //记录node在堆上的位置
    std::unordered_map<Node*,int> heapIndexMap;
    std::unordered_map<Node*,int> distanceMap;
    int heapSize = 0;


    bool isInHeap(Node* node){
        return isEntered(node) && (heapIndexMap[node] != -1);
    }

    void swap(int index1,int index2){
        if(index1 == index2) return ;
        heapIndexMap[nodes[index1]] = index2;
        heapIndexMap[nodes[index2]] = index1;
        Node* temp = nodes[index1];
        nodes[index1] = nodes[index2];
        nodes[index2] = temp;
    }

    void heapInsert(Node* node,int index){ //向上换
        while (index >0 && distanceMap[nodes[index]] < distanceMap[nodes[(index - 1) / 2]])
        {
            swap(index,(index-1)/2);
            index = (index-1)/2;
        }
        
    }

    void heapify(int index){ //向下沉
        int size = nodes.size();
        int left = index*2 + 1;
        int count = 1;
        while (left < size){
            int smallest = left + 1 < size && distanceMap[nodes[left + 1]] < distanceMap[nodes[left]]
            ? left + 1 : left; //????
            smallest = distanceMap[nodes[smallest]] < distanceMap[nodes[index]] ? smallest : index;
            if(smallest == index) break;
            swap(smallest,index);
            index = smallest;
            left = index*2 + 1;
        }                                                                                            
    }

    public:
    bool isEntered(Node* node){
        return heapIndexMap.count(node);
    }

    minHeap(){};

    bool empty(){
        return nodes.empty();
    }

    void doNode(Node* last,Node* node, int distance){
        //update
        if(isInHeap(node)){
            distanceMap[node] = std::min(distanceMap[node],distance);
            heapInsert(node,heapIndexMap[node]);
            if(distanceMap[node] >= distance){
                node->father = last;
            }
            std::cout << "update done,node:" << node ->val << std::endl;
        }
        //add
        if(!isEntered(node)){
            nodes.push_back(node);
            std::cout<<"此处为插入节点，插入后元素个数为" << nodes.size() <<std::endl;
            heapIndexMap[node] = nodes.size() - 1;
            distanceMap[node] = distance;
            heapInsert(node,nodes.size() - 1);
            node->father = last;
            std::cout << "add done,node:" << node ->val << std::endl;
        }
        
    }

    nodeRecord pop(){
        int size = nodes.size();
        nodeRecord myRecord = nodeRecord(nodes[0],distanceMap[nodes[0]]);
        swap(0,size - 1);
        heapIndexMap[nodes[size - 1]] = -1;
        distanceMap.erase(nodes[size - 1]);
        nodes.pop_back();
        heapify(0);
        return myRecord;
    }

    int size(){
        return nodes.size();
    }

};


class solution{
    public:
    static Graph createGraph(const std::vector<std::vector<int>>& matrix) {
        Graph graph;
        for (const auto& edgeInfo : matrix) {
            int weight = edgeInfo[0];
            int from = edgeInfo[1];
            int to = edgeInfo[2];

            // Create or get the 'from' node
            auto resultFrom = graph.nodes.emplace(from, new Node(from));
            Node* fromNode = resultFrom.first->second;

            // Create or get the 'to' node
            auto resultTo = graph.nodes.emplace(to, new Node(to));
            Node* toNode = resultTo.first->second;

            // Create the edge and update node information
            Edge* newEdge = new Edge(weight, fromNode, toNode);
            fromNode->nexts.push_back(toNode);
            fromNode->edges.push_back(newEdge);
            graph.edges.insert(newEdge);
        }
        return graph;
    }

    static void BFS(Node* start) {
		std::queue<Node*> q;
		std::set<Node*> s;
		q.push(start);
		s.insert(start);
		while (!q.empty()) {
			Node* cur = q.front();
			q.pop();
			std::cout << cur->val << " ";
			for (auto i : cur->nexts) {
				if (s.find(i) == s.end()) {
					q.push(i);
					s.insert(i);
				}
			}
		}
	}
    static void DFS(Node* start) {
		std::stack<Node*> st;  
		std::set<Node*> s;
		s.insert(start);
		st.push(start);
		std::cout << start->val << " ";
		while (!st.empty()) {
			Node* cur = st.top();
			st.pop();
			for (auto i : cur->nexts) {
				if (s.find(i) == s.end()) {
					st.push(cur);
					st.push(i);
					s.insert(i);
					std::cout << i->val << " ";
					break;
				}
			}
		}
	}



    static std::unordered_map<Node*,int> dijkstraPlus(Node* start){
        minHeap nodeHeap;
        //std::cout<<"此处为插入头结点前,此时nodes中元素个数:"<< nodeHeap.size()<<std::endl;
        nodeHeap.doNode(nullptr,start,0);
        //std::cout<<"此处为插入头结点,此时nodes中元素个数:"<< nodeHeap.size()<<std::endl;
        std::unordered_map<Node*,int> myAns;
        while (!nodeHeap.empty()){
            nodeRecord myRecord = nodeHeap.pop();
            Node* cur = myRecord.node;
            int distance = myRecord.distance;
            //std::cout <<"节点 "<< cur->val << "  的distance:" << distance<<std::endl;
            for(Edge* edge : cur -> edges){
                nodeHeap.doNode(cur,edge->to,edge->weight + distance); 
            }
            myAns[cur] = distance;
        }
        return myAns;
    }

};


int main(){
    std::vector<std::vector<int>> matrix = {{1,1,2},{6,1,4},{2,2,3},{5,2,5},{1,3,7},{5,3,10},{1,4,6},{4,4,5},{1,5,10},
                                {8,6,10},{1,7,4},{4,7,8},{2,8,9},{1,8,6},{3,9,5},{5,9,10}};
    std::vector<std::vector<int>> matrix_search = {{1,1,2},{2,2,3},{3,1,4},{1,1,5},{2,2,6},{1,2,7},{3,2,8},{1,3,6},
                                {3,3,7},{5,3,8},{2,4,6},{6,4,7},{1,4,8},{1,5,6},{3,5,7},{1,5,8},{5,6,9},{1,7,9},{2,8,9}};
    Graph myGraph = solution::createGraph(matrix);
    Node* start = myGraph.nodes[4];//选择值为1的节点作为开始节点
    // solution::DFS(start);
    // cout << endl;
    // solution::BFS(start);
    std::unordered_map<Node*, int> result = solution::dijkstraPlus(start);
    for (const auto& entry : result) {
        std::cout << "Shortest distance to Node " << entry.first->val << ": " << entry.second << std::endl;
    }
    Node* cur = myGraph.nodes[10];
    std::vector<int> path; 
    while(cur != nullptr){
        path.push_back(cur->val);
        cur = cur->father;
    }
    std::reverse(path.begin(),path.end());
    for(int i : path){
        std::cout << i << " ";
    }
    return 0;
}