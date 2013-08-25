//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 7 July 2013

#include <iostream>
#include "Pathfinder.h"
#include "GraphGen.h"
#include <fstream>
#include "boost/graph/graphviz.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"

using namespace boost;
using namespace std;
using namespace ADStar;

int main(){

    GraphGen g;
    //g.convert("bun_zipper.ply", "bun_zipper.adg");
    g.read("bun_zipper.adg");

    int start = 34000, end = 1400;

    Pathfinder pf;
    pf.findPath(start, end, g.g);

    vector<Pathfinder::WeightedGraph::vertex_descriptor> p(num_vertices(g.g));
    vector<int> d(num_vertices(g.g));

    dijkstra_shortest_paths(g.g, vertex(start, g.g),
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(vertex_index, g.g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(vertex_index, g.g))));

    std::cout << "Djikstra distance(" << start << ", " << end << ") = " << d[end] << std::endl;

/*
    //Trivial case
    Pathfinder::WeightedGraph graph;
    add_edge(0, 1, 20, graph);
    add_edge(0, 2, 2, graph);
    add_edge(1, 3, 20, graph);
    add_edge(2, 3, 1, graph);
    pf.findPath(0, 3, graph);

    //Failure test
    Pathfinder::WeightedGraph graph4;
    add_edge(0, 1, 20, graph4);
    remove_edge(0,1, graph4);
    pf.findPath(0, 1, graph4);

    //Second-most trivial case
    Pathfinder::WeightedGraph graph2;
    add_edge(0, 1, 20, graph2);
    add_edge(0, 2, 2, graph2);
    add_edge(1, 3, 2, graph2);
    add_edge(2, 3, 30, graph2);
    pf.findPath(0, 3, graph2);

    int start = 0, end = 7;

    //Non-trivial
    Pathfinder::WeightedGraph graph3;
    add_edge(0,1,20, graph3);
    add_edge(0,2,4, graph3);
    add_edge(0,3,19, graph3);
    add_edge(1,4,15, graph3);
    add_edge(1,5,7, graph3);
    add_edge(2,4,70, graph3);
    add_edge(2,6,1, graph3);
    add_edge(3,5,4, graph3);
    add_edge(3,6,10, graph3);
    add_edge(4,7,30, graph3);
    add_edge(5,7,80, graph3);
    add_edge(6,7,15, graph3);
    add_vertex(Pathfinder::vertex_props(), graph3);
    pf.findPath(start,end, graph3);

    vector<Pathfinder::WeightedGraph::vertex_descriptor> p(num_vertices(graph3));
    vector<int> d(num_vertices(graph3));

    dijkstra_shortest_paths(graph3, vertex(start, graph3),
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(vertex_index, graph3))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(vertex_index, graph3))));*/


   // Pathfinder::WeightedGraph::vertex_iterator vi, vend;
  //for (boost::tie(vi, vend) = vertices(graph3); vi != vend; ++vi) {
    //std::cout << "distance(" << end << ") = " << d[end] << std::endl;
    //std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::endl;
  //}

    /*ofstream og("graph.dot");
    dynamic_properties dp;
    dp.property("id", get(vertex_index,graph3));
    dp.property("weight", get(edge_weight,graph3));
    write_graphviz_dp(og, graph3, dp, "id");

    Pathfinder::WeightedGraph ing;
    read_graphviz("graph.dot", ing, dp, "id");*/

}
