#ifndef MESH_ADAPTOR_H
#define MESH_ADAPTOR_H

#include "plyParser.h"

typedef Polyhedron::Halfedge_handle Halfedge_handle; 
typedef Polyhedron::Edge_iterator Edge_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Facet_circulator;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector_3;

class MeshAdaptor{
	

public:	
	Polyhedron P;

	MeshAdaptor(){};
	void init(char * fileName);	
	void split_cell(Facet_circulator h);
	void split_triangles();
	int longestEdge(Facet_circulator h);
	Facet_circulator greatest_area_triangle();
	float triangle_area(Facet_circulator h);
	bool triangle_is_sliver(const Point * pts, Facet_circulator h);
	void bisect_sliver(int halfedgeIndex, Point midEdge, Halfedge_handle h);
	
	Polyhedron getSurface(){return P;}


};

#endif MESH_ADAPTOR_H