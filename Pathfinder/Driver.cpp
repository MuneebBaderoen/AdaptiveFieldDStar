//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 7 July 2013

#include <iostream>
#include "Pathfinder.h"
#include "GraphGen.h"
#include "NodeUpdator.h"
#include <fstream>
#include "boost/graph/dijkstra_shortest_paths.hpp"

using namespace boost;
using namespace std;
using namespace ADStar;

int main(){

    GraphGen g;
    //g.convert("bun_zipper.ply", "bun_zipper.adg");
    g.read("bun_zipper.adg");

    int start = 34000, end = 1400;
    //int start = 34280, end = 34420;

    NodeUpdator nu;

    Pathfinder pf(g.g);
    pf.findPath(start, end);

    /*nu.changeEdge(34001, 34138, 8);

    nu.changeEdge(34003, 34004, 20000);
    nu.changeEdge(34138, 34139, 100);
    nu.changeEdge(34139, 34140, 100);
    nu.changeEdge(33222, 33223, 500);*/

    while(!pf.atEnd()){

        pf.move();
        pf.handleChanges(nu.getChanges());
        nu.clear();

    }

    pf.printPath();

    vector<Pathfinder::WeightedGraph::vertex_descriptor> p(num_vertices(g.g));
    vector<int> d(num_vertices(g.g));

    dijkstra_shortest_paths(g.g, vertex(start, g.g),
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(vertex_index, g.g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(vertex_index, g.g))));

    std::cout << "Djikstra distance(" << start << ", " << end << ") = " << d[end] << std::endl;

    //pf.findPath(start, end);
    /*int dStart = 34000, dEnd = 1283;
    pf.findPath(dStart, dEnd);

    while(!pf.atEnd()){

        pf.move();
        pf.handleChanges(nu.getChanges());
        nu.clear();

    }

    pf.printPath();

    dijkstra_shortest_paths(g.g, vertex(dStart, g.g),
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(vertex_index, g.g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(vertex_index, g.g))));

    std::cout << "Djikstra distance(" << dStart << ", " << dEnd << ") = " << d[dEnd] << std::endl;*/



}
