//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 22 August 2013

#ifndef PATHFINDER_H_INCLUDED
#define PATHFINDER_H_INCLUDED

#include "boost/graph/adjacency_list.hpp"
#include "boost/heap/d_ary_heap.hpp"
#include <iostream>

namespace ADStar{

class Pathfinder{

    public:

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

        struct vertex_props{node_key key; float x, y, z; };

        typedef boost::property<boost::edge_weight_t, float> WeightProp;
        typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS
                    , vertex_props, WeightProp> WeightedGraph;
        typedef typename boost::graph_traits<WeightedGraph>::out_edge_iterator oe_iterator;
        typedef typename boost::graph_traits<WeightedGraph>::in_edge_iterator ie_iterator;
        typedef typename boost::heap::d_ary_heap<Node, boost::heap::mutable_<true>
                                , boost::heap::arity<4>
                                , boost::heap::compare<std::greater<Node> >
                                > Dary_queue;
        typedef typename std::pair<int, Dary_queue::handle_type> handle_pair;

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
        float km;
        std::map<int, Dary_queue::handle_type> on_queue;
        Dary_queue p_queue;
        bool changed;

};

}

#endif // PATHFINDER_H_INCLUDED
