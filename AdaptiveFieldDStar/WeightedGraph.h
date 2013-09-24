//typedefs for weighted graph structure
//Daanyaal du Toit
//Created: 8 September 2013
//Last modified: 8 September 2013

#ifndef WEIGHTEDGRAPH_H_INCLUDED
#define WEIGHTEDGRAPH_H_INCLUDED

#include "boost/graph/adjacency_list.hpp"

namespace ADStar{

    struct graph_key : public std::pair<float, float>{

        graph_key(): std::pair<float, float>(INFINITY, INFINITY){}

    };

    struct vertex_props{graph_key key; float x, y, z; };

    typedef boost::property<boost::edge_weight_t, float> WeightProp;
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS
                    , vertex_props, WeightProp> WeightedGraph;

}

#endif // WEIGHTEDGRAPH_H_INCLUDED
