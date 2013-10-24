#include "meshAdaptor.h"

int MeshAdaptor::longestEdge(Facet_circulator h){
	return 0;
}

Polyhedron::Vertex_handle MeshAdaptor::split_on_edge(Halfedge_handle h, Point midpt){

    using namespace std;

    Halfedge_handle opp = h->opposite()->next();
    Polyhedron::Vertex_handle nvh = bisect_sliver(0, midpt, h);

    std::cout << "Splitting on: " << opp->vertex()->point() << ", " << opp->next()->next()->vertex()->point() << std::endl;
    std::cout << "facet: " << opp->vertex()->point() << ", "
                << opp->next()->vertex()->point() << ", "
                << opp->next()->next()->vertex()->point() << ", "
                << opp->next()->next()->next()->vertex()->point()
                << " degree" << opp->facet()->facet_degree() << std::endl;

    std::cout << "facet degree: " << opp->facet()->facet_degree() << std::endl;
    std::cout << "Opp vert: " << opp->vertex()->point() << std::endl;
    std::cout << "Next Opp vert: " << opp->next()->vertex()->point() << std::endl;
    std::cout << "Next Next Opp vert: " << opp->next()->next()->vertex()->point() << std::endl;

	Polyhedron::Halfedge_handle diag = P.split_facet(opp, opp->next()->next());

	std::cout << "facet: " << diag->vertex()->point() << ", "
                << diag->next()->vertex()->point() << ", "
                << diag->next()->next()->vertex()->point() << ", "
                << diag->next()->next()->next()->vertex()->point()
                << " degree" << diag->facet()->facet_degree() << std::endl;

    std::cout << "facet: " << diag->opposite()->vertex()->point() << ", "
                << diag->opposite()->next()->vertex()->point() << ", "
                << diag->opposite()->next()->next()->vertex()->point() << ", "
                << diag->opposite()->next()->next()->next()->vertex()->point()
                << " degree" << diag->facet()->facet_degree() << std::endl;

    return nvh;

}


std::pair<bool, Polyhedron::Vertex_handle> MeshAdaptor::triangle_is_sliver(const Point * pts, Facet_circulator hfc){
	using namespace std;

	float longestEdge=0, shortestEdge=0;
	int longestIndex = -1;

	for(int i = 0; i<3;++i){
		int next = (i+1+3)%3;
		Vector_3 v(pts[i].x()-pts[next].x(),
			pts[i].y()-pts[next].y(),
			pts[i].z()-pts[next].z());

		float currentEdgeLength = v.squared_length();

		if(currentEdgeLength>longestEdge){
			longestEdge=currentEdgeLength;
			longestIndex=next;

			if(shortestEdge==0)
				shortestEdge=longestEdge;
		}
		else if(currentEdgeLength<shortestEdge){
			shortestEdge=currentEdgeLength;
		}

	}
	cout<<"longestEdge: "<<longestEdge<<", shortestEdge: "<<shortestEdge<<endl;

	//First bisection condition
	if(longestEdge/shortestEdge>=2){
		cout<<"bisecting triangle"<<endl;
		int prev = (longestIndex-1+3)%3;
		Point mid(pts[prev].x()/2+pts[longestIndex].x()/2,
			pts[prev].y()/2+pts[longestIndex].y()/2,
			pts[prev].z()/2+pts[longestIndex].z()/2);

		Halfedge_handle h = hfc;
		for(int i = 0; i<longestIndex; ++i)
			h=hfc->next();

		return std::pair<bool, Polyhedron::Vertex_handle>(true, bisect_sliver(longestIndex, mid, h));
	}

	return std::pair<bool, Polyhedron::Vertex_handle>(false, Polyhedron::Vertex_handle());
}

Polyhedron::Vertex_handle MeshAdaptor::bisect_sliver(int halfedgeIndex, Point midEdge, Halfedge_handle h){
	//Incoming handle is the vertex adjacent to vertices incident with the edge to be added
	//thus next and previous verts are connected, splitting the facet

    std::cout << "Mesh is valid: " << (bool)P.is_valid() << std::endl;
    std::cout << "Splitting edge: " << h->vertex()->point() << ", " << h->opposite()->vertex()->point() << std::endl;

	Halfedge_handle newH = P.split_edge(h);
	newH->vertex()->point() = midEdge;
	std::cout << "diag dest: " << newH->vertex()->point() << ", " << h->next()->vertex()->point() << std::endl;
	Polyhedron::Halfedge_handle diag = P.split_facet(newH, h->next());

	std::cout << "New edge is along " << newH->vertex()->point() << " , " << newH->opposite()->vertex()->point() << std::endl;
	std::cout << "Facet: " << newH->vertex()->point() << ", " << newH->next()->vertex()->point()
                << ", " << newH->next()->next()->vertex()->point()
                << " (degree)" << newH->facet()->facet_degree()
                << " (weight)" << newH->facet()->weight << std::endl;

	std::cout << "Old edge is along " << h->vertex()->point() << " , " << h->opposite()->vertex()->point() << std::endl;
    std::cout << "Facet: " << h->vertex()->point() << ", " << h->next()->vertex()->point()
                << ", " << h->next()->next()->vertex()->point()
                << " (degree)" << h->facet()->facet_degree()
                << " (weight)" << h->facet()->weight << std::endl;


	std::cout << "Mesh is valid: " << (bool)P.is_valid() << std::endl;

	return h->prev()->vertex();
}


Polyhedron::Vertex_handle MeshAdaptor::find_halfedge_handle(PathPoly_3::Vertex p1, PathPoly_3::Vertex p2, Point midPnt){
    using namespace ADStar;
    for(Polyhedron::Halfedge_iterator it = P.halfedges_begin(); it!=P.halfedges_end(); ++it){
        if(isEqual(*(it->vertex()), p1))
            if(isEqual(*(it->opposite()->vertex()), p2)){
                std::cout << "Found" << p1.point() << " , " << p2.point() << std::endl;
                return split_on_edge(it, midPnt);}
        if(isEqual(*(it->vertex()), p2))
            if(isEqual(*(it->opposite()->vertex()), p1)){
                std::cout << "Found" << p2.point() << " , " << p1.point() << std::endl;
                return split_on_edge(it, midPnt);}
    }

    std::cout << "No edge found" << std::endl;
    return Polyhedron::Vertex_handle();
}


Polyhedron::Vertex_handle MeshAdaptor::split_cell(Facet_circulator hfc){
	using namespace std;

	Point * points = new Point[3];

	points[0] = hfc->vertex()->point();
	points[1] = hfc->next()->vertex()->point();
	points[2] = hfc->next()->next()->vertex()->point();

    std::pair<bool, Polyhedron::Vertex_handle> isSliver = triangle_is_sliver(points, hfc);

	if(!isSliver.first){

		Point sum(points[0].x()+points[1].x()+points[2].x(),
			points[0].y()+points[1].y()+points[2].y(),
			points[0].z()+points[1].z()+points[2].z());

		//Add center vertex, split into 3
		Halfedge_handle c1 = P.create_center_vertex(hfc);
		c1->vertex()->point() = Point(sum.x()/3, sum.y()/3, sum.z()/3);
		return c1->vertex();

	}

	return isSliver.second;

}
Facet_circulator MeshAdaptor::greatest_area_triangle(){
	Facet_circulator h;
	float greatest_area = 0;
	for(Polyhedron::Facet_iterator it = P.facets_begin(); it!=P.facets_end(); ){
		if(triangle_area(it->facet_begin())>greatest_area)
			h=it->facet_begin();
	}
	return h;
}

float MeshAdaptor::triangle_area(Facet_circulator hfc){
	float area = 0;
	//insert cgal area method here

	Point * points = new Point[3];

	points[0] = hfc->vertex()->point();
	points[1] = hfc->next()->vertex()->point();
	points[2] = hfc->next()->next()->vertex()->point();

	Vector_3 v1(points[0].x()-points[1].x(),
		points[0].y()-points[1].y(),
		points[0].z()-points[1].z());

	Vector_3 v2(points[0].x()-points[2].x(),
		points[0].y()-points[2].y(),
		points[0].z()-points[2].z());

	Vector_3 normal = CGAL::cross_product<Kernel>(v1,v2);
	Vector_3 altitude = CGAL::cross_product<Kernel>(v1,normal);



	return area;
}




void MeshAdaptor::split_triangles(){
		//for(Polyhedron::Facet_iterator it = P.facets_begin(); it!=P.facets_end(); ){
			Polyhedron::Facet_iterator it = P.facets_begin();
			split_cell(it->facet_begin());

			//split_cell(it->facet_begin());

			//it++;it++;it++;it++;


		//}

	}


