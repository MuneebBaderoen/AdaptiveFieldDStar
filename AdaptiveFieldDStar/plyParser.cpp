#include <iostream>
#include <sstream>
#include <fstream>
#include <string>


#include "plyParser.h"


PlyParser::PlyParser(){

}

std::vector<Triangle> PlyParser::readPlyFile(char* fileName, float scale){

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
						Vertex v(x,y,z);
						v*=scale;
						vertices.push_back(Vector3(v.x,v.y,v.z));
						
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

	/*
	for(std::vector<Vertex>::iterator iter = vertices.begin(); iter!=vertices.end();++iter){
		
		cout<<"Sum of faces: ";
		cout<<(*iter).x;
		cout<<(*iter).y;
		cout<<(*iter).z;
		(*iter).normalize();
		cout<<"\nNormalized: ";
		cout<<(*iter).x;
		cout<<(*iter).y;
		cout<<(*iter).z;
		cout<<endl;
	}
	*/

	//cout<<"Vector referencing"<<vertices[0].x<<endl;
	/*
	for(int i = 0; i<numVerts; ++i){
		cout<<"Sum of faces: ";
		cout<<vertices[i].x;
		cout<<vertices[i].y;
		cout<<vertices[i].z;
		vertices[i].normalize();
		cout<<"\nNormalized: ";
		cout<<vertices[i].x;
		cout<<vertices[i].y;
		cout<<vertices[i].z;
		cout<<endl;
		//cout<<PlyParser::vertices[i].normal<<endl;
	}
	*/
	//Triangle t1 = triangles.at(20);

	cout<<"HEY"<<endl;
	return triangles;
	
}


