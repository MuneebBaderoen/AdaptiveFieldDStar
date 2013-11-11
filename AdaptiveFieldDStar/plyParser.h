#ifndef PLY_PARSER_H
#define PLY_PARSER_H

#include "helperClasses.h"
#include "Helpers.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>

typedef ADStar::K     Kernel;
typedef ADStar::PathPoly_3         Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

template<class HDS>
class PlyParser : public CGAL::Modifier_base<HDS>{

public:
	int numVerts;
	int numFaces;

	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;
	std::map<int, std::vector<int> > edges;

	PlyParser(char * fileName){readPlyFile(fileName, 1);}
	std::vector<Triangle> readPlyFile(char* fileName, float scale){

	numVerts=0;
	numFaces=0;

	int tenPercentVerts = 0;
	int tenPercentFaces = 0;
	using namespace std;
	fstream inFile(fileName);
	string line;
	//istringstream iss;
	if (inFile.is_open())
	{
		//ply
		getline (inFile,line);
		if(line=="ply")
			cout<<"This is a ply file"<<endl;

		//format ascii
		getline (inFile,line);

		int count = 0;

		bool onVertices = false;
		bool onFaces = false;

		while ( inFile.good() )
		{
			getline (inFile,line);
			istringstream iss(line);
			string word;
			iss>>word;
			if(word=="comment")
				continue;
			else if(word=="element"){
				iss>>word;
				int num;
				iss>>num;
				if(word=="vertex"){
					//vertices=new Vertex[num];
					onVertices=true;
					numVerts=num;
					tenPercentVerts=numVerts/10;
					cout<<"numVerts: "<<num<<endl;
				}
				if(word=="face"){
					//triangles=new Triangle[num];
					numFaces=num;
					tenPercentFaces=numFaces/10;
					cout<<"numFaces: "<<num<<endl;
				}

			}
			else{
				if(word=="property"){
					continue;
				}
				if(word=="end_header"){
					continue;
				}

				//cout<<line<<endl;
				if(onVertices)
				{
					if(count==numVerts){
						count=0;
						onFaces=true;
						onVertices=false;

					}
					if(!onFaces){
						float x, y, z;
						//cout<<count++<<endl;
						//word is now actually the xCo-ordinate
						istringstream numParse(word);
						numParse>>x;
						iss>>y;
						iss>>z;

						count++;
						if(count%tenPercentVerts==0)
							cout<<count<<"/"<<numVerts<<"  Vertex: "<<x<<","<<y<<","<<z<<endl;
						Vertex v(count-1, x,y,z);
						v*=scale;
						vertices.push_back(Vertex(v));

					}
				}
				if(onFaces)
				{
					if(count>=numFaces)
						break;
					count++;
					if(count%tenPercentFaces==0)
						cout<<count<<"/"<<numFaces<<" Face: "<<line<<endl;

					int p1,p2,p3;
					iss>>p1>>p2>>p3;

					Triangle t(	vertices[p1],
								vertices[p2],
								vertices[p3]
								);
					t.calculateNormal();
					t.calculateLongestEdge();


					vertices[p1].normal+=t.normal;
					vertices[p2].normal+=t.normal;
					vertices[p3].normal+=t.normal;

					if(t.v1.longestEdge>vertices[p1].longestEdge)
						vertices[p1].longestEdge=t.v1.longestEdge;
					if(t.v2.longestEdge>vertices[p2].longestEdge)
						vertices[p2].longestEdge=t.v2.longestEdge;
					if(t.v3.longestEdge>vertices[p3].longestEdge)
						vertices[p3].longestEdge=t.v3.longestEdge;

					triangles.push_back(t);

				}
			}
			//cout << line << endl;
		}
		inFile.close();
		cout<<"File closed"<<endl;
	}
	else{
		cout<<"not open"<<endl;
	}

	cout<<"HEY"<<endl;
	return triangles;

}

    void operator()(HDS & hds){

        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( numVerts, numFaces);
        typedef typename Polyhedron::HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
        for(int i = 0; i < numVerts; ++i)
            B.add_vertex( Point( vertices[i].x, vertices[i].y, vertices[i].z));
        for(int i = 0; i < numFaces; ++i){
            B.begin_facet();
            B.add_vertex_to_facet( triangles[i].v1.index);

            bool connected = false;

            for(int j = 0; j < edges[triangles[i].v1.index].size(); ++j)
            if(edges[triangles[i].v1.index][j] == triangles[i].v2.index)
                {connected = true; break;}

            if(!connected)
            for(int j = 0; j < edges[triangles[i].v3.index].size(); ++j)
                if(edges[triangles[i].v3.index][j] == triangles[i].v1.index)
                    {connected = true; break;}

            if(!connected)
            for(int j = 0; j < edges[triangles[i].v2.index].size(); ++j)
                if(edges[triangles[i].v2.index][j] == triangles[i].v3.index)
                    {connected = true; break;}

            if(connected){       // if 1 connected to 2
            //std::cout << "already connected " << std::endl;

            B.add_vertex_to_facet( triangles[i].v3.index);
            B.add_vertex_to_facet( triangles[i].v2.index);
            edges[triangles[i].v1.index].push_back(triangles[i].v3.index);
            if(i == 147 || i == 292)
                std::cout << "Swapped edges " << triangles[i].v1.index << " "
                << triangles[i].v3.index << " " << triangles[i].v2.index << std::endl;
            edges[triangles[i].v3.index].push_back(triangles[i].v2.index);
            edges[triangles[i].v2.index].push_back(triangles[i].v1.index);
            B.end_facet();
            }
            else{  //if 1 connected to 3 or free
            //std::cout << "connecting " << std::endl;
            if(i == 147 || i == 292)
                std::cout << "Added edges " << triangles[i].v1.index << " "
                << triangles[i].v2.index << " " << triangles[i].v3.index << std::endl;
            B.add_vertex_to_facet( triangles[i].v2.index);
            B.add_vertex_to_facet( triangles[i].v3.index);
            edges[triangles[i].v1.index].push_back(triangles[i].v2.index);
            edges[triangles[i].v2.index].push_back(triangles[i].v3.index);
            edges[triangles[i].v3.index].push_back(triangles[i].v1.index);
            B.end_facet();
            }

            }
        B.end_surface();
        std::cout << "Done building" << std::endl;

    }

};

#endif
