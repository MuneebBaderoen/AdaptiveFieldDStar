#ifndef MESH_ADAPTOR_H
#define MESH_ADAPTOR_H

#include "plyParser.h"

typedef Polyhedron::Halfedge_handle Halfedge_handle;
typedef Polyhedron::Edge_iterator Edge_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Facet_circulator;
typedef ADStar::K::Point_3 Point;
typedef ADStar::K::Vector_3 Vector_3;

class MeshAdaptor{


public:
	Polyhedron P;

	MeshAdaptor(){};
	void init(char * filename){
	using namespace std;
	PlyParser <HalfedgeDS> parser (filename);
	P.delegate(parser);

	cout<<"Number of verts: "<<P.size_of_vertices()<<endl;
    }

    Polyhedron::Vertex_handle split_cell(Facet_circulator h);
	void split_triangles();
	int longestEdge(Facet_circulator h);
	Facet_circulator greatest_area_triangle();
	float triangle_area(Facet_circulator h);
	std::pair<bool, Polyhedron::Vertex_handle> triangle_is_sliver(const Point * pts, Facet_circulator h);
	Polyhedron::Vertex_handle bisect_sliver(int halfedgeIndex, Point midEdge, Halfedge_handle h);
	void split_on_edge(Halfedge_handle h, Point midpt);

	Polyhedron getSurface(){return P;}


};

#endif
