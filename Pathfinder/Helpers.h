//Helpers for pathfinding
//Daanyaal du Toit
//Created: 23 September 2013
//Last Modified: 23 September 2013

#ifndef HELPERS_H_INCLUDED
#define HELPERS_H_INCLUDED

#include "CGAL/Polyhedron_incremental_builder_3.h"
#include "CGAL/Polyhedron_3.h"
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"

namespace ADStar{

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Vector_3<K> Vec3;

struct node_key : public std::pair<float,float>{

            node_key(): std::pair<float,float>(INFINITY,INFINITY){}
            node_key(float g, float rhs): std::pair<float,float>(g, rhs){}
            bool operator <(const node_key & nk) const{

                if(std::fabs(first - nk.first) < 1e-5 || (std::isinf(first) && std::isinf(nk.first)))
                    return second < nk.second;
                else
                    return first < nk.second;


            }

            friend std::ostream & operator <<(std::ostream& out, const node_key & key){

                out << "(" << key.first << ", " << key.second << ")";
                return out;

            }

        };

//Custom vert properties for pathfinder
template<class Refs, class P>
struct My_Vert : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, P>{

    My_Vert(){}
    My_Vert(const P & p): CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, P>(p){}
    node_key key;

};

//Custom edge properties for D* Lite
template<class Refs>
struct My_HalfEdge : public CGAL::HalfedgeDS_halfedge_base<Refs>{

    float weight;

};

//Custom face properties for Field D*
template<class Refs>
struct My_Face : public CGAL::HalfedgeDS_face_base<Refs>{

    float weight;

};

struct DStar_items : public CGAL::Polyhedron_items_3{

    template<class Refs, class Traits>
    struct Vertex_wrapper{typedef My_Vert<Refs, typename Traits::Point_3> Vertex;};

    /*template<class Refs, class Traits>
    struct Halfedge_wrapper{typedef My_HalfEdge<Refs> Halfedge;};*/

    template<class Refs, class Traits>
    struct Face_wrapper{typedef My_Face<Refs> Face;};

};

typedef CGAL::Polyhedron_3<K, DStar_items> PathPoly_3;

bool isEqual(PathPoly_3::Vertex v1, PathPoly_3::Vertex v2){

        return v1.point().x() == v2.point().x() && v1.point().y() == v2.point().y() && v1.point().z() == v2.point().z();

    }

class Build_temp : public CGAL::Modifier_base<PathPoly_3::HDS> {

    PathPoly_3::Vertex v;
    PathPoly_3::Halfedge_handle he;
    PathPoly_3::Facet_handle T1, T2;

public:
    Build_temp(PathPoly_3::Vertex _v, PathPoly_3::Halfedge_handle _he, PathPoly_3::Facet_handle _T1, PathPoly_3::Facet_handle _T2)
                : v(_v), he(_he), T1(_T1), T2(_T2){}

    void operator()(PathPoly_3::HDS & hds) {

        CGAL::Polyhedron_incremental_builder_3<PathPoly_3::HDS> B(hds, true);
        B.begin_surface(5, 2);
        PathPoly_3::Vertex_handle vh = B.add_vertex( v.point());

        CGAL::Point_3<K> p1 = he->vertex()->point(), p2 = he->opposite()->vertex()->point();
        float len = sqrt(Vec3(p1, p2).squared_length());
        float len1 = sqrt(Vec3(p1,v.point()).squared_length());
        float len2 = sqrt(Vec3(p2,v.point()).squared_length());

        std::cout << "Length of " << he->vertex()->point() << ", " << he->opposite()->vertex()->point() << " = " << len << std::endl;

        vh->key.first = (len1/len)*he->vertex()->key.first + (len2/len)*he->opposite()->vertex()->key.first;

        //add shared edge vertices
        vh = B.add_vertex(he->vertex()->point());
        vh->key = he->vertex()->key;
        vh = B.add_vertex(he->opposite()->vertex()->point());
        vh->key = he->opposite()->vertex()->key;

        //add remaining vertices and cells
        PathPoly_3::Facet_handle f;
        PathPoly_3::Halfedge_around_facet_circulator fit1 = T1->facet_begin();
        do{

            if(!isEqual(*(fit1->vertex()), *(he->vertex())) && !isEqual(*(fit1->vertex()), *(he->opposite()->vertex()))){

                vh = B.add_vertex(fit1->vertex()->point());
                vh->key = fit1->vertex()->key;

                f = B.begin_facet();
                B.add_vertex_to_facet( 0);
                B.add_vertex_to_facet( 1);
                B.add_vertex_to_facet( 3);
                B.end_facet();
                f->weight = T1->weight;

                f = B.begin_facet();
                B.add_vertex_to_facet( 0);
                B.add_vertex_to_facet( 3);
                B.add_vertex_to_facet( 2);
                B.end_facet();
                f->weight = T1->weight;

                break;

            }
            ++fit1;

        }
        while(fit1 != T1->facet_begin());

        if(T2 != PathPoly_3::Facet_handle()){

        PathPoly_3::Halfedge_around_facet_circulator fit2 = T2->facet_begin();
        do{

            if(!isEqual(*(fit2->vertex()), *(he->vertex())) && !isEqual(*(fit2->vertex()), *(he->opposite()->vertex()))){

                vh = B.add_vertex(fit2->vertex()->point());
                vh->key = fit2->vertex()->key;

                f = B.begin_facet();
                B.add_vertex_to_facet( 1);
                B.add_vertex_to_facet( 0);
                B.add_vertex_to_facet( 4);
                B.end_facet();
                f->weight = T2->weight;

                f = B.begin_facet();
                B.add_vertex_to_facet( 0);
                B.add_vertex_to_facet( 2);
                B.add_vertex_to_facet( 4);
                B.end_facet();
                f->weight = T2->weight;

                break;

            }
            ++fit2;

        }
        while(fit2 != T2->facet_begin());

    }
        B.end_surface();
    }
};

}

#endif // HELPERS_H_INCLUDED
