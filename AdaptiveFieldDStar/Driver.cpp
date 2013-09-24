//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 23 July 2013

#include <iostream>
#include "Pathfinder.h"
#include "NodeUpdator.h"
#include <fstream>
#include "meshAdaptor.h"

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

    PathPoly_3 mesh;
    Build_triangle<PathPoly_3::HDS> tri;
    mesh.delegate(tri);

    MeshAdaptor adapt;

    PathPoly_3::Vertex_handle start(mesh.vertices_begin());
    PathPoly_3::Vertex_handle end((--mesh.vertices_end()));

    adapt.P = mesh;

    cout << "From " << start->point() << " to " << end->point() << endl;

    Pathfinder<Node<PathPoly_3::Vertex_handle>, PathPoly_3> pfC(adapt.P);
    pfC.findPath(start, end);

    int y = 0;
    /*while(y++ < 3){

        vector<PathPoly_3::Vertex_handle> newVerts;
        vector<PathPoly_3::Facet_handle> toProcess = pfC.getPath();
        for(vector<PathPoly_3::Facet_handle>::iterator it = toProcess.begin(); it != toProcess.end(); ++it)
            newVerts.push_back(adapt.split_cell(it->facet_begin()));
        pfC.handleChanges(newVerts);

    }*/

    pfC.getPath();

}
