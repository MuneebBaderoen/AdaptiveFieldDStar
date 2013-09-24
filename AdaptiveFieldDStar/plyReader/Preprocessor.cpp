//converts PLY file into splat list
//Daanyaal du Toit
//02 May 2013

#include "Preprocessor.h"

namespace QSplat{

//Adapted from plytest.c
void Preprocessor::read(std::string fileName){

    std::cout << "Reading " << fileName << std::endl;

    PlyFile ply;
    int numElemTypes;
    int numElems;
    int nProps;
    char ** elemsList;
    int fileType;
    float ver;

    //check ply file information
    ply = *ply_open_for_reading(fileName.c_str(), &numElemTypes, &elemsList, &fileType, &ver);

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

            for(int j = 0; j < numElems; ++j){

                Vertex vert;
                ply_get_element(&ply, &vert);

                Splat splat;
                splat.center = vert;
                splats.push_back(splat);

                if(vert.x > bb.maxX)
                    bb.maxX = vert.x;
                else if(vert.x < bb.minX)
                    bb.minX = vert.x;

                if(vert.y > bb.maxY)
                    bb.maxY = vert.y;
                else if(vert.y < bb.minY)
                    bb.minY = vert.y;

                if(vert.z > bb.maxZ)
                    bb.maxZ = vert.z;
                else if(vert.z < bb.minZ)
                    bb.minZ = vert.z;

            }

        }
        else if(equal_strings(curElem, "face")){

            //prepare to read in face elements
            ply_get_property (&ply, curElem, &face_props[0]);
            ply_get_property (&ply, curElem, &face_props[1]);

            for(int j = 0; j < numElems; ++j){

                Face face;
                ply_get_element(&ply, &face);
                Splat bs;

                //create vector for cross product
                Vector vec1(splats[face.verts[0]].center - splats[face.verts[1]].center);
                Vector vec2(splats[face.verts[1]].center - splats[face.verts[2]].center);

                //get normal and center of bounding sphere for face
                for(int k = 0; k < face.nverts; ++k){
                    bs.center += splats[face.verts[k]].center;
                    splats[face.verts[k]].normal += vec1.cross(vec2);

                }

                bs.center /= face.nverts;

                //get radius of bounding sphere for face
                for(int k = 0; k < face.nverts; ++k){

                    if(bs.center.dist(splats[face.verts[k]].center) > bs.radius)
                        bs.radius = bs.center.dist(splats[face.verts[k]].center);

                }

                //store max radius per vertex
                for(int k = 0; k < face.nverts; ++k)
                    if(splats[face.verts[k]].radius < bs.radius)
                        splats[face.verts[k]].radius = bs.radius;

            }

        }

        delete [] curElem;

    }

    delete [] elemsList;

    for(std::vector<Splat>::iterator it = splats.begin(); it != splats.end(); ++it){

        it->normal.normalise();

        if(it->radius < minRad)
            minRad = it->radius;

    }

    ply_close(&ply);
    root = buildTree(bb, splats);

    }

    //write splats as is
    void Preprocessor::write(std::string fileName){

        std::cout << "Writing raw splat file to " << fileName << std::endl;

        std::ofstream fileOut(fileName);
        for(std::vector<Splat>::iterator it = splats.begin(); it != splats.end(); ++it){

            fileOut << *it << std::endl;

        }

        splats.clear();

    }

    //writes bounding heirarchy of splats
    void Preprocessor::writeTree(std::string fileName){

        std::cout << "Writing breadth-first splat hierarchy to " << fileName << std::endl;

        std::ofstream fileOut(fileName);
        std::deque<Splat *> queue;

        queue.push_back(root);

        while(!queue.empty()){

            fileOut << *queue.front() << std::endl;
            queue.front()->addToQueue(queue);
            queue.pop_front();

        }

    }

}
