#ifndef PLY_PARSER_H
#define PLY_PARSER_H

#include "helperClasses.h"
#include <vector>

class PlyParser{
	
public:
	int numVerts;
	int numFaces;

	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;

	PlyParser();	
	std::vector<Triangle> readPlyFile(char* fileName,float scale);
};

#endif