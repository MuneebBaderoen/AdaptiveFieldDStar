
#include <iostream>
#include "Pathfinder.h"

using namespace boost;
using namespace std;
using namespace ADStar;

int main(){

    Pathfinder pf;

    //Trivial case
    Pathfinder::WeightedGraph graph;
    add_edge(0, 1, 20, graph);
    add_edge(0, 2, 2, graph);
    add_edge(1, 3, 20, graph);
    add_edge(2, 3, 1, graph);
    pf.findPath(0, 3, graph);

    //Second-most trivial case
    Pathfinder::WeightedGraph graph2;
    add_edge(0, 1, 20, graph2);
    add_edge(0, 2, 2, graph2);
    add_edge(1, 3, 2, graph2);
    add_edge(2, 3, 30, graph2);
    pf.findPath(0, 3, graph2);

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
    pf.findPath(0,7, graph3);

}
