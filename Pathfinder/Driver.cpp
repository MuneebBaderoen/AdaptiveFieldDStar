//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 23 July 2013

#include <iostream>
#include "Pathfinder.h"
#include "GraphGen.h"
#include "NodeUpdator.h"
#include <fstream>
#include "boost/graph/dijkstra_shortest_paths.hpp"

using namespace boost;
using namespace std;
using namespace ADStar;

template <class HDS>
class Build_triangle : public CGAL::Modifier_base<HDS> {
public:
    Build_triangle() {}
    void operator()(HDS & hds) {
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( 7, 5);
        typedef typename PathPoly_3::HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
        B.add_vertex( Point( 0, 0, 0));
        B.add_vertex( Point( 1, 0, 0));
        B.add_vertex( Point( -1, 0, 0));
        B.add_vertex( Point( 0, 1, 0));
        B.add_vertex( Point( -2, 1, 0));
        B.add_vertex( Point( 2, 3, 0));
        B.add_vertex( Point( 5, 1, 0));
        PathPoly_3::Facet_handle f = B.begin_facet();
        B.add_vertex_to_facet( 0);
        B.add_vertex_to_facet( 1);
        B.add_vertex_to_facet( 3);
        B.end_facet();
        f->weight = 1;
        f = B.begin_facet();
        B.add_vertex_to_facet( 0);
        B.add_vertex_to_facet( 3);
        B.add_vertex_to_facet( 2);
        B.end_facet();
        f->weight = 1;
        f = B.begin_facet();
        B.add_vertex_to_facet( 2);
        B.add_vertex_to_facet( 3);
        B.add_vertex_to_facet( 4);
        B.end_facet();
        f->weight = 1;
        f = B.begin_facet();
        B.add_vertex_to_facet( 3);
        B.add_vertex_to_facet( 1);
        B.add_vertex_to_facet( 5);
        B.end_facet();
        f->weight = 1;
        f = B.begin_facet();
        B.add_vertex_to_facet( 5);
        B.add_vertex_to_facet( 1);
        B.add_vertex_to_facet( 6);
        B.end_facet();
        f->weight = 1;
        B.end_surface();
    }
};

int main(){

    //GraphGen g;
    //g.convert("bun_zipper.ply", "bun_zipper.adg");
    //g.read("bun_zipper.adg");

    //int start = 34000, end = 1400;
    //int start = 34280, end = 34420;

    //NodeUpdator nu;

    PathPoly_3 mesh;
    Build_triangle<PathPoly_3::HDS> tri;
    mesh.delegate(tri);

    PathPoly_3::Vertex_handle start(mesh.vertices_begin());
    PathPoly_3::Vertex_handle end((--mesh.vertices_end()));

    cout << "From " << start->point() << " to " << end->point() << endl;

    //Pathfinder<Node<int>, WeightedGraph> pf(g.g);
    Pathfinder<Node<PathPoly_3::Vertex_handle>, PathPoly_3> pfC(mesh);
    pfC.findPath(start, end);

    //nu.changeEdge(34001, 34138, 8);

    //nu.changeEdge(34003, 34004, 20000);
    /*nu.changeEdge(34138, 34139, 100);
    nu.changeEdge(34139, 34140, 100);
    nu.changeEdge(33222, 33223, 500);*/

    /*while(!pfC.atEnd()){

        pfC.move();
        //pf.handleChanges(nu.getChanges());
        //nu.clear();

    }*/



    pfC.printPath();

    /*vector<WeightedGraph::vertex_descriptor> p(num_vertices(g.g));
    vector<int> d(num_vertices(g.g));

    dijkstra_shortest_paths(g.g, vertex(start, g.g),
                          predecessor_map(boost::make_iterator_property_map(p.begin(), get(vertex_index, g.g))).
                          distance_map(boost::make_iterator_property_map(d.begin(), get(vertex_index, g.g))));

    std::cout << "Djikstra distance(" << start << ", " << end << ") = " << d[end] << std::endl;*/

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
