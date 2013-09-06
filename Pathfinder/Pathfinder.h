//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 04 September 2013

#ifndef PATHFINDER_H_INCLUDED
#define PATHFINDER_H_INCLUDED

#include "boost/graph/adjacency_list.hpp"
#include "boost/heap/d_ary_heap.hpp"
#include "UpdateHelpers.h"
#include <iostream>
#include <iomanip>

namespace ADStar{

class Pathfinder{

    struct node_key : public std::pair<float,float>{

            node_key(): std::pair<float,float>(INFINITY,INFINITY){}
            node_key(float g, float rhs): std::pair<float,float>(g, rhs){}
            bool operator <(const node_key & nk) const{

                if(std::fabs(first - nk.first) < 1e-5)
                    return second < nk.second;
                else
                    return first < nk.second;


            }

            friend std::ostream & operator <<(std::ostream& out, const node_key & key){

                out << "(" << key.first << ", " << key.second << ")";
                return out;

            }

        };

    struct Node{

            int index;                      //graph handle for the current vertex
            node_key key;                   //the g and rhs value of the vertex

            Node(int i, node_key k): index(i), key(k){}
            Node(int i, float g, float rhs): index(i), key(g, rhs){}  //initial node constructor

            bool operator < (const Node & n) const{

                return key < n.key;

            }

            bool operator > (const Node & n) const{

                return n.key < key;

            }

        };

    public:

        struct vertex_props{node_key key; float x, y, z; };

        typedef boost::property<boost::edge_weight_t, float> WeightProp;
        typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS
                    , vertex_props, WeightProp> WeightedGraph;
        typedef typename boost::graph_traits<WeightedGraph>::out_edge_iterator oe_iterator;
        typedef typename boost::graph_traits<WeightedGraph>::in_edge_iterator ie_iterator;
        typedef typename boost::heap::d_ary_heap<Node, boost::heap::mutable_<true>
                                , boost::heap::arity<4>
                                , boost::heap::compare<std::greater<Node > >
                                > Dary_queue;
        typedef typename std::pair<int, Dary_queue::handle_type> handle_pair;

        Pathfinder(WeightedGraph & g): graph(g){}

//---------------------------------------------------------------------------------
//Main pathfinding method
//---------------------------------------------------------------------------------

    void findPath(int s, int e){

        cost = 0;

        //initialise pathfinder
        start = s;
        end = e;

        last = start;
        path.clear();
        on_queue.clear();
        p_queue.clear();
        km = 0;
        graph[end].key.second = 0;
        graph[end].key.first = heuristic(start, end);

        Dary_queue::handle_type h = p_queue.push(Node(end, heuristic(start, end), 0));
        on_queue[end] = h;

        //perform initial path computation
        computePath();

    }

//---------------------------------------------------------------------------------
//Move along the least expensive path
//---------------------------------------------------------------------------------

    void move(){

        if(start != end){

        //exit if no path
        if(graph[start].key.second == INFINITY){

            std::cout << "No path found." << std::endl;
            return;

        }

        /*if(start == 34003)
            std::cout << "Key of 34002 : " << graph[34002].key << std::endl;*/

        //calc min traversal to one of cur's successors and assign to start
        std::pair<oe_iterator, oe_iterator> suc_range = boost::out_edges(start, graph);

        float min_cost = INFINITY;
        int min_succ;

        for(oe_iterator it = suc_range.first; it != suc_range.second; ++it){

            int cur_index = boost::target(*it, graph);

            /*if(start == 34002)
                std::cout << "Succ: " << cur_index << " g = " << graph[cur_index].key.first << " edge cost = "
                    << boost::get(boost::edge_weight, graph, *it) << " Sum: "
                    << graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it) << std::endl;*/

            if(graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it) < min_cost){

                min_cost = graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it);
                min_succ = cur_index;

            }

        }

        //std::cout << "Min Succ = " << min_succ << " g = " << graph[min_succ].key.first;
        //std::cin >> a;

        //move to succesor
        path.push_back(start);
        cost += boost::get(boost::edge_weight, graph, edge(start, min_succ, graph).first);
        //std::cout << "Cost from " << start << " to " << min_succ << " = " << min_cost - graph[min_succ].key.first << std::endl;
        //std::cout << "Accum Cost = " << cost << std::endl;
        start = min_succ;
        //std::cout << "Moving to " << start << std::endl;

        if(start == end)
            path.push_back(start);

        }

        //std::cout << "Move complete" << std::endl;

    }

//---------------------------------------------------------------------------------
//Handle changes polled by the NodeUpdator
//---------------------------------------------------------------------------------

    void handleChanges(const std::vector<EdgeChange> & c){

        using namespace boost;

        if(!c.empty()){

            //std::cout << "Change detected" << std::endl;

            km += heuristic(last,start);
            last = start;
            //for all changed (directed) edges
            for(std::vector<EdgeChange>::const_iterator it = c.begin(); it != c.end(); ++it){

                //std::cout << "Processing start = " << it->start << ", end = " << it->end << std::endl;
                //std::cout << "Edge exists " << edge(it->start, it->end, graph).second << std::endl;

                WeightedGraph::edge_descriptor e = edge(it->start, it->end, graph).first;
                float old_weight = get(edge_weight, graph, e);

                //std::cout << "Old weight: " << get(edge_weight, graph, e) << std::endl;

                //update edge cost
                put(edge_weight, graph, e, it->weight);

                //std::cout << "Change edge weight from " << old_weight << " to " << get(edge_weight, graph, e) << std::endl;

                //std::cout << "start.rhs " << std::setprecision(10) << graph[it->start].key.second << std::endl;
                //std::cout << "end.g + old " << graph[it->end].key.first + old_weight << std::endl;

                //std::cout << "end.rhs " << graph[it->end].key.second << std::endl;
                //std::cout << "start.g + old " << graph[it->start].key.first + old_weight << std::endl;

                if(it->start != end){

                    if(old_weight > it->weight){//std::cout << "Weight lowered." << std::endl;
                        graph[it->start].key.second = std::min(graph[it->start].key.second, it->weight + graph[it->end].key.first);}
                    else if(std::fabs(graph[it->start].key.second - (old_weight + graph[it->end].key.first)) == 0){//std::cout << "Accurate lookahead" << std::endl;
                        graph[it->start].key.second = minSucCost(it->start);}

                }
                /*if(it->end != end){

                    if(old_weight > it->weight){std::cout << "Weight lowered." << std::endl;
                        graph[it->end].key.second = std::min(graph[it->end].key.second, it->weight + graph[it->start].key.first);}
                    else if(std::fabs(graph[it->end].key.second - (old_weight + graph[it->start].key.first)) == 0){std::cout << "Accurate lookahead" << std::endl;
                        graph[it->end].key.second = minSucCost(it->end);}

                }*/

                //std::cout << "Start key: " << graph[it->start].key << std::endl;

                //update node
                updateNode(it->start);
                //updateNodeD(it->end);

            }

            //std::cout << "Computing path" << std::endl;
            computePath();

        }

    }

//---------------------------------------------------------------------------------
//Get path and cost of path
//---------------------------------------------------------------------------------

    void printPath(){

        std::cout << "Shortest Path:" << std::endl;
        for(std::vector<int>::iterator it = path.begin(); it != path.end(); ++it)
            std::cout << *it << " ";
        std::cout << std::endl;

        std::cout << "Path cost: " << cost << std::endl;

    }
        inline bool atEnd(){return start == end;}

    private:

        //Pathfinder members
        std::vector<int> path;
        WeightedGraph & graph;
        int last, start, end;
        float km, cost;
        std::map<int, Dary_queue::handle_type> on_queue;
        Dary_queue p_queue;

//---------------------------------------------------------------------------------
//Compute priority key of node
//---------------------------------------------------------------------------------

    node_key computeKey(int index){

    return node_key(std::min(graph[index].key.first, graph[index].key.second) + heuristic(start, index) + km
                    , std::min(graph[index].key.first, graph[index].key.second));

    }

//---------------------------------------------------------------------------------
//Change the key of graph node
//---------------------------------------------------------------------------------

    void updateNode(int index){

        if(graph[index].key.first != graph[index].key.second){  //if key is inconsistent
            if(on_queue.find(index) != on_queue.end()){         //if already on queue
            //update node priority

            //graph[index].key = computeKey(index);
            p_queue.update(on_queue[index], Node(index, computeKey(index)));
            //std::cout << "Updating key for node " << index << " to " << graph[index].key << std::endl;

            }
            else{
            //insert node onto queue
            //graph[index].key = computeKey(index);
            Dary_queue::handle_type h = p_queue.push(Node(index, computeKey(index)));
            on_queue[index] = h;
            //std::cout << "Inserting node " << index << " with key " << graph[index].key << std::endl;
            }

        }
        else if(on_queue.find(index) != on_queue.end()){        //if key consistent and on queue
            //remove node from queue
            p_queue.erase(on_queue[index]);
            on_queue.erase(index);
            //std::cout << "Removing node " << index << std::endl;

        }

    }

//---------------------------------------------------------------------------------
//Finds a path from the goal to the start
//---------------------------------------------------------------------------------

    void computePath(){

    while(!p_queue.empty() && (p_queue.top().key < computeKey(start) || graph[start].key.second > graph[start].key.first)){
    //while(!p_queue.empty() && (p_queue.top().key < computeKey(start) || graph[start].key.second - graph[start].key.first > 1e-5)){

        //std::cout << "Processing Node " << p_queue.top().index << std::endl;

        //compare old and new keys for changes
        int top_index = p_queue.top().index;
        node_key key_old = p_queue.top().key;
        node_key key_new = computeKey(top_index);

        //std::cout << "Old key of " << top.index << " : " << key_old << std::endl;
        //std::cout << "New key of " << top.index << " : " << key_new << std::endl;

        //if key has changed
        if(key_old < key_new){

            //update the current node on the queue
            //std::cout << "Updating node " << top.index << " with key " << key_new.first << ", " << key_new.second << std::endl;
            p_queue.update(on_queue[top_index], Node(top_index, key_new));

            }
        //if the graph's key has different values
        else if(graph[top_index].key.first > graph[top_index].key.second){

            //std::cout << "Adjusting inconsistent key of " << top.index << std::endl;
            //update node and remove from queue
            graph[top_index].key.first = graph[top_index].key.second;
            p_queue.pop();
            on_queue.erase(top_index);

            std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top_index, graph);

            //for all predecesors
            for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

                int cur_index = boost::source(*it, graph);
                if(cur_index != end){           //if that node is not goal

                    graph[cur_index].key.second = std::min(graph[cur_index].key.second, boost::get(boost::edge_weight, graph, *it)  + graph[top_index].key.first); //rhs = min g and cost of any successor
                    //std::cout << "Set key of " << cur_index << " to " << graph[cur_index].key << std::endl;

                }

            //update node
            //std::cout << "Update Node " << cur_index << std::endl;
            updateNode(cur_index);
            }

        }
        else{

        float old_g = graph[top_index].key.first;
        graph[top_index].key.first = INFINITY;

        //std::cout << "Key of " << top.index << " : " << graph[top.index].key << std::endl;

        std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top_index, graph);

        //for all predecesors and cur node
        for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

            int cur_index = boost::source(*it, graph);
            //if that node rhs = cost to that node from cur + g_cur
            if(graph[cur_index].key.second == boost::get(boost::edge_weight, graph, *it) + old_g)
                if(cur_index != end){           //if that node is not goal

                    graph[cur_index].key.second = minSucCost(cur_index); //rhs = min g and cost of any successor

                }

            //update node
            updateNode(cur_index);
            }

        if(graph[top_index].key.second == old_g)
            if(top_index != end)           //if that node is not goal
                graph[top_index].key.second = minSucCost(top_index); //rhs = min g and cost of any successor

        updateNode(top_index);

        }

    }

    }

//---------------------------------------------------------------------------------
//Estimates the distance between two vertices
//---------------------------------------------------------------------------------

    float heuristic(int s_index, int e_index){

        return sqrtf(pow(graph[e_index].x - graph[s_index].x, 2) + pow(graph[e_index].y - graph[s_index].y, 2)
                     + pow(graph[e_index].z - graph[s_index].z, 2));

    }

//---------------------------------------------------------------------------------
//Calculate the min cost to a successor node
//---------------------------------------------------------------------------------

    float minSucCost(int index){

        std::pair<oe_iterator, oe_iterator> suc_range = boost::out_edges(index, graph);
        float min_cost = INFINITY;

        for(oe_iterator it = suc_range.first; it != suc_range.second; ++it){

            int suc_index = boost::target(*it, graph);
            if(min_cost > boost::get(boost::edge_weight, graph, *it) + graph[suc_index].key.first)
                min_cost = boost::get(boost::edge_weight, graph, *it) + graph[suc_index].key.first;

        }

        return min_cost;

    }

};

}

#endif // PATHFINDER_H_INCLUDED
