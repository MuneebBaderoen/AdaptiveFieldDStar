//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 7 July 2013

#ifndef PATHFINDER_H_INCLUDED
#define PATHFINDER_H_INCLUDED

#include "boost/graph/adjacency_list.hpp"
#include "boost/heap/d_ary_heap.hpp"
#include <iostream>

namespace ADStar{

class Pathfinder{

    public:

        typedef std::pair<float, float> node_key;

        struct Node{

            int index;                      //graph handle for the current vertex
            node_key key;                   //the g and rhs value of the vertex

            Node(int i, node_key k): index(i), key(k){}
            Node(int i, float g, float rhs): index(i), key(g, rhs){}  //initial node constructor

        };

        struct NodeComp{

            bool operator()(const Node & a, const Node & b) const{

            return a.key.first < b.key.first;

            }

        };

        struct vertex_props{node_key key; vertex_props(): key(INFINITY, INFINITY){}};

        typedef boost::property<boost::edge_weight_t, float> WeightProp;
        typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS
                    , vertex_props, WeightProp> WeightedGraph;
        typedef typename boost::graph_traits<WeightedGraph>::out_edge_iterator oe_iterator;
        typedef typename boost::graph_traits<WeightedGraph>::in_edge_iterator ie_iterator;
        typedef typename boost::heap::d_ary_heap<Node, boost::heap::mutable_<true>
                                , boost::heap::arity<4>
                                //, boost::heap::stable<false>
                                , boost::heap::compare<NodeComp>
                                > Dary_queue;
        typedef typename std::pair<int, Dary_queue::handle_type> handle_pair;

        //Pathfinder(WeightedGraph & g): graph(g){};
        void findPath(int start, int end, WeightedGraph & g);

    private:
        //private functions
        node_key computeKey(int index);
        void computePath();
        void updateNode(int index);
        float heuristic(int s_index, int e_index);
        //float minPredCost(int index);
        float minSucCost(int index);

        //Pathfinder members
        std::vector<int> path;
        WeightedGraph graph;
        int start, end;
        float km = 0;
        std::map<int, Dary_queue::handle_type> on_queue;
        Dary_queue p_queue;


};

}

#endif // PATHFINDER_H_INCLUDED
