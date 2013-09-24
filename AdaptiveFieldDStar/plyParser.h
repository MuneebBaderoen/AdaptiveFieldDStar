#ifndef PLY_PARSER_H
#define PLY_PARSER_H

#include<fstream>
#include<vector>
#include<string>
#include<algorithm>
#include "Helpers.h"

#include "helperClasses.h"
#include "plyReader\ply.h"
/*
#include<CGAL/Simple_cartesian.h>
#include<CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/Polyhedron_3.h>
#include<CGAL/IO/Polyhedron_iostream.h>
*/


typedef ADStar::K     Kernel;
typedef ADStar::PathPoly_3         Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

template<class HDS>
class PlyParser : public CGAL::Modifier_base<HDS>{
	std::vector<Vertex> vertices;
	std::vector<int>    triangles;

	int numVerts, numFaces;
public:

    //PlyParser( std::vector<double> &v, std::vector<int> &t ) : vertices(v), triangles(t) {}

	PlyParser(){}
	PlyParser(char * fileName){
		using namespace std;
		cout<<fileName<<endl;
		readFile(fileName);

	}



	void readFile(char * fileName){
		PlyProperty vert_props[] = { /* list of property information for a vertex */
		  {"x", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,x), 0, 0, 0, 0},
		  {"y", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,y), 0, 0, 0, 0},
		  {"z", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,z), 0, 0, 0, 0},
		};

		PlyProperty face_props[] = { /* list of property information for a vertex */
		  {"vertex_indices", PLY_INT, PLY_INT, offsetof(Triangle,verts),
			1, PLY_UCHAR, PLY_UCHAR, offsetof(Triangle,nverts)}
		};



		using namespace std;


		PlyFile ply;
		int numElemTypes;
		int numElems;
		int nProps;
		char ** elemsList;
		int fileType;
		float ver;

		//check ply file information
		ply = *ply_open_for_reading(fileName, &numElemTypes, &elemsList, &fileType, &ver);

		 //read in elements
		for(int i = 0; i < numElemTypes; ++i){

			char * curElem = elemsList[i];
			ply_get_element_description (&ply, curElem, &numElems, &nProps);

			//handle vertices
			if(equal_strings(curElem, "vertex")){

				//prepare to read in
				ply_get_property(&ply, curElem, &vert_props[0]);
				ply_get_property(&ply, curElem, &vert_props[1]);
				ply_get_property(&ply, curElem, &vert_props[2]);


				numVerts = numElems;
				for(int j = 0; j < numElems; ++j){

					Vertex vert;
					ply_get_element(&ply, &vert);
					vertices.push_back(vert);
					//cout<<vert<<endl;

				}

			}
			else if(equal_strings(curElem, "face")){

				//prepare to read in face elements
				ply_get_property (&ply, curElem, &face_props[0]);
				numFaces = numElems;

				for(int j = 0; j < numElems; ++j){

					Triangle face;
					ply_get_element(&ply, &face);
					triangles.push_back(face.verts[0]);
					triangles.push_back(face.verts[1]);
					triangles.push_back(face.verts[2]);
					/*
					cout<<"<<<"
						<<face.v1<<", \n"
						<<face.v2<<", \n"
						<<face.v3<<">>>"<<endl;
						*/
				}

			}

			delete [] curElem;

		}
		delete [] elemsList;

		//ply_close(&ply);
	}

	void operator()(HDS& hds){
		using namespace std;
		std::cout<<"Entering void operator"<<std::endl;
		cout<<"Numverts: "<<numVerts<<", Numfaces: "<<numFaces<<endl;

		typedef typename HDS::Vertex   HDSVertex;
        typedef typename HDSVertex::Point Point;

		// create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( numVerts, numFaces );

		// add the vertices
		for(vector<Vertex>::iterator it= vertices.begin(); it!=vertices.end();it++){
			B.add_vertex( Point( (*it).x,(*it).y,(*it).z ) );
		}

		// add the triangles
		for( int i=0; i<(int)triangles.size(); i+=3 ){
			B.begin_facet();
			B.add_vertex_to_facet( triangles[i+0] );
			B.add_vertex_to_facet( triangles[i+1] );
			B.add_vertex_to_facet( triangles[i+2] );
			B.end_facet();
		}

		B.end_surface();
	}



};

#endif
