#define _USE_MATH_DEFINES
#include <Gl/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>


#include "meshAdaptor.h"
#include "Pathfinder.h"

#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace boost;
using namespace std;
using namespace ADStar;


MeshAdaptor adapt;
PathPoly_3 mesh;


int currentFace = 0;
int windowX = 800, windowY = 600;



bool cameraTrue = true;

Vector3 modelMin, modelMax;
Vector3 modelPos = Vector3(0,0,0);

//float cx = 0.0f, cy = 0.0f, cz = 0.0f,
float crx = 120.0f, cry = 60.0f, zoom = 10;

Vector3 camPos;
Vector3 camUp = Vector3(0,1,0);

float inc = 0.1f;

int mousePosX, mousePosY;
int oldMousePosX, oldMousePosY;
bool mouseClicked = false;
bool interactiveMode = false;

Halfedge_handle currentHalfEdge;
int currentEdgeCounter = 5;
bool performSplit = false;


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
        //B.add_vertex( Point( 2, 3, 0));
        //B.add_vertex( Point( 5, 1, 0));
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
        /*f = B.begin_facet();
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
        f->weight = 1;*/
        B.end_surface();
    }
};




void resetCam()
{
	//camPos=Vector3(0,0,zoom);
	crx = 120.0f;
	cry = 60.0f;
	zoom =75;
}



void camera()
{
	/*
	//glTranslatef(camPos.x,camPos.y,camPos.z);
	//rotations of the camera
	glRotatef(cry, 1.0f,0.0f,0.0f);
	glRotatef(crx, 0.0f,1.0f,0.0f);

	//translations of the camera
	glTranslatef(-camPos.x, -10, -camPos.z);
	*/
	camPos.z=modelPos.z-zoom;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, windowX/windowY, 1.0, 10000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//glRotatef(-cry, 1.0f,0.0f,0.0f);
	//glRotatef(crx, 0.0f,1.0f,0.0f);

	float radianCrx = crx/180*M_PI;
	float radianCry = cry/180*M_PI;

	camPos.x=sin(radianCry)*cos(radianCrx)*zoom;
	camPos.z=sin(radianCry)*sin(radianCrx)*zoom;
	camPos.y=cos(radianCry)*zoom;

	//std::cout<<"Frustum position: "<<viewFrustum.position<<", Camera Position: "<<camPos<<std::endl;
	gluLookAt(	camPos.x, camPos.y, camPos.z,
				modelPos.x, modelPos.y, modelPos.z,
				camUp.x, camUp.y, camUp.z);


}
void mouseClick(int button, int state, int x, int y){
	using namespace std;

	if(button==0){
		if(state==0){

		}

	}


}
void mouseDrag(int x, int y){
	using namespace std;
	interactiveMode=true;
	mousePosX=x;
	mousePosY=y;

	short inc = 3.0f;
	short Xinc = inc;
	short Yinc = -inc;


	if(mousePosX<oldMousePosX)
		crx-= Xinc;
	if(mousePosX>oldMousePosX)
		crx+= Xinc;
	if(mousePosY<oldMousePosY)
		cry-= Yinc;
	if(mousePosY>oldMousePosY)
		cry+= Yinc;


	if(cry>135)
		cry=135;
	if(cry<30)
		cry=30;


	oldMousePosX=mousePosX;
	oldMousePosY=mousePosY;


}
/*Point getCurrentEdgeMidPt(){



    return new Point();
}
*/

Point getCurrentEdgeMidPt(){
    cout<<"getting midpnt"<<endl;
    Point p1 = currentHalfEdge->vertex()->point();

    cout<<"immediate fail"<<endl;
    Point p2 = currentHalfEdge->opposite()->vertex()->point();

    Point out(p1.x()/2+p2.x()/2,
    p1.y()/2+p2.y()/2,
    p1.z()/2+p2.z()/2);

    return out;

}

void keyb(unsigned char key, int x, int y){

	if(key=='s'){
		adapt.split_triangles();
	}
	if(key=='d'){
		performSplit=true;
	}
	if(key=='r'){
		resetCam();
	}
	if(key=='6'){
		currentFace++;
		currentFace%=adapt.getSurface().size_of_facets();
	}
	if(key=='4'){
		currentFace--;
		currentFace+=adapt.getSurface().size_of_facets();
		currentFace%=adapt.getSurface().size_of_facets();
	}
    if(key=='3'){
		currentEdgeCounter++;
		currentEdgeCounter%=adapt.getSurface().size_of_halfedges();
	}
	if(key=='1'){
		currentEdgeCounter--;
		currentEdgeCounter+=adapt.getSurface().size_of_halfedges();
		currentEdgeCounter%=adapt.getSurface().size_of_halfedges();
	}
	if(key=='-'){
		zoom+=0.5f;
	}
	if(key=='+'){
		zoom-=0.5f;
	}
}
void reshape(int width, int height){

	glViewport(0,0,(GLsizei) width,(GLsizei) height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat) width/(GLfloat) height,1.0, 100);

	glMatrixMode(GL_MODELVIEW);
}
void Text(const char * text, int length, int x, int y){
	glColor3f(1.0, 1.0, 1.0);
	glMatrixMode(GL_PROJECTION);
	// lol is a matrix array
	double *lol = new double[16];
	glGetDoublev(GL_PROJECTION_MATRIX, lol);
	glLoadIdentity();
	glOrtho(0,800,0,600,-5,5);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2i(x,y);

	for(int a = 0;a < length;++a)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, (int)text[a]);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(lol);
	glMatrixMode(GL_MODELVIEW);
}
void drawLines(){

	glColor3f(1.0, 1.0, 1.0);

	//X axis
	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3d(50,0,0);
	glVertex3d(-50,0,0);



	//Y axis

	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(0,50,0);
	glVertex3d(0,-50,0);



	//Z axis

	glColor3f(1.0, 0.0, 0.0);
	glVertex3d(0,0,50);
	glVertex3d(0,0,-50);

	glEnd();

	glColor3f(0.0,0.0,0.0);
}
void drawCube(Vector3& min, Vector3& max){

	glColor3f(1.0, 1.0, 1.0);

	//X axis
	glBegin(GL_LINES);
	glColor3f(1.0, 1.0, 0.0);
	glVertex3d(min.x,min.y,min.z);
	glVertex3d(min.x,min.y,max.z);

	glVertex3d(min.x,min.y,min.z);
	glVertex3d(max.x,min.y,min.z);

	glVertex3d(max.x,min.y,min.z);
	glVertex3d(max.x,min.y,max.z);

	glVertex3d(max.x,min.y,max.z);
	glVertex3d(min.x,min.y,max.z);

	//----------------------------------

	glVertex3d(min.x,min.y,min.z);
	glVertex3d(min.x,max.y,min.z);

	glVertex3d(max.x,min.y,min.z);
	glVertex3d(max.x,max.y,min.z);

	glVertex3d(min.x,min.y,max.z);
	glVertex3d(min.x,max.y,max.z);

	glVertex3d(max.x,min.y,max.z);
	glVertex3d(max.x,max.y,max.z);

	//----------------------------------

	glVertex3d(min.x,max.y,min.z);
	glVertex3d(min.x,max.y,max.z);

	glVertex3d(min.x,max.y,min.z);
	glVertex3d(max.x,max.y,min.z);

	glVertex3d(max.x,max.y,min.z);
	glVertex3d(max.x,max.y,max.z);

	glVertex3d(max.x,max.y,max.z);
	glVertex3d(min.x,max.y,max.z);

	glEnd();


}
void calculateBoundingBox(Vector3 center, float radius){
	Vector3 bbMin = Vector3(center.x-radius,center.y-radius,center.z-radius);
	Vector3 bbMax = Vector3(center.x+radius,center.y+radius,center.z+radius);
	drawCube(bbMin, bbMax);

}





void display(){

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(1.0, 1.0, 1.0);

	//Text(getText().data(), getText().size(),5,25);
	//Text(getText2().data(), getText2().size(),5,5);

	camera();

	//drawLines();



	int count = 0;
	glColor3f(1.0, 1.0, 1.0);




	/*
	for(Polyhedron::Edge_iterator iter = P.edges_begin(); iter!=P.edges_end();	++iter){
		count++;

		//HalfedgeDS::Vertex_handle v1, v2;
		//v1=(*iter).vertex(0);

		if(count <=3)
			glColor3f(1.0f,1.0f,1.0f);
		else
			glColor3f(1.0f,0.2f,0.2f);
		const Point& a = iter->vertex()->point();
		const Point& b = iter->opposite()->vertex()->point();

		//std::cout<<a.x()<<", "<<a.y()<<", "<<a.z()<<std::endl;

		glBegin( GL_LINES);
		glVertex3f(a.x(), a.y(), a.z());
		glVertex3f(b.x(), b.y(), b.z());
		glEnd();



	}
	*/

	Polyhedron & P = adapt.getSurface();
	int edgeCount = 0;
	for(Polyhedron::Halfedge_iterator it = P.halfedges_begin(); it!=P.halfedges_end(); ++it){
        edgeCount++;
        if(edgeCount-1==currentEdgeCounter){
            Point p1 = it->vertex()->point();
            Point p2 = it->opposite()->vertex()->point();
            Point out (p1.x()/2+p2.x()/2,
            p1.y()/2+p2.y()/2,
            p1.z()/2+p2.z()/2);
            glColor3f(0.0f,1.0f,0.0f);
            if(performSplit)
                adapt.split_on_edge(it, out);

            performSplit=false;


            int dist = 1;

			glBegin( GL_LINES);
			glVertex3f(p2.x(), p2.y(), p2.z());
			glVertex3f(p1.x(), p1.y(), p1.z());
			glEnd();

        }


	}



	for(Polyhedron::Facet_iterator it = P.facets_begin(); it!=P.facets_end();++it)
	{
		count++;



		Polyhedron::Halfedge_around_facet_circulator pCirc = it->facet_begin();
		Polyhedron::Halfedge_handle h = pCirc;

		Vector3 pt(0,0,0);



		for(int i = 0; i<(int)it->facet_degree(); i++){
			pt.x+=h->vertex()->point().x();
			pt.y+=h->vertex()->point().y();
			pt.z+=h->vertex()->point().z();
			h=h->next();
		}
		pt.x/=(int)it->facet_degree();
		pt.y/=(int)it->facet_degree();
		pt.z/=(int)it->facet_degree();


		h = pCirc;
		for(int i = 0; i<(int)it->facet_degree(); i++){
		if(count-1==currentFace)
			glColor3f(1.0f,1.0f,1.0f);
		else
			glColor3f(1.0f,0.2f,0.2f);


			const Point a = h->vertex()->point();
			const Point b = h->opposite()->vertex()->point();

			//std::cout<<a.x()<<", "<<a.y()<<", "<<a.z()<<std::endl;
			/*
			glBegin( GL_LINES);
			glVertex3f(a.x(), a.y(), a.z());
			glVertex3f(b.x(), b.y(), b.z());
			glEnd();
			*/
			int dist = 5;

			glBegin( GL_LINES);
			glVertex3f(a.x()+(pt.x-a.x())/dist,
				a.y()+(pt.y-a.y())/dist,
				a.z()+(pt.z-a.z())/dist);
			glVertex3f(b.x()+(pt.x-b.x())/dist,
				b.y()+(pt.y-b.y())/dist,
				b.z()+(pt.z-b.z())/dist);
			glEnd();



			//if((int)it->facet_degree()!=3)
			/*
			glColor3f(0.2f,1.0f,0.2f);
			glBegin( GL_LINES);
			glVertex3f(a.x(), a.y(), a.z());
			glVertex3f(pt.x, pt.y, pt.z);

			glEnd();
			*/


			h=h->next();
		}



	}
	//cout<<count<<endl;


	//cout<<m.getSurface().size_of_facets()<<endl;





	glFlush();



}





int main(int argc, char** argv)
{
    Build_triangle<PathPoly_3::HDS> tri;


    mesh.delegate(tri);

    PathPoly_3::Vertex_handle start((mesh.vertices_begin()));
    PathPoly_3::Vertex_handle end((--mesh.vertices_end()));
    adapt.P = mesh;
	//std::string filename = "Resources/bunny_recon/bun_zipper.ply";
	//std::string filename = "Resources/tetra.ply";
	//m.init((char *) filename.c_str());

	camPos=Vector3(0,0,0-zoom);


    cout << "From " << start->point() << " to " << end->point() << endl;

    Pathfinder<Node<PathPoly_3::Vertex_handle>, PathPoly_3> pfC(adapt.P);
    pfC.findPath(start, end);

    int y = 0;
       while(y++ < 2){

        deque<PathPoly_3::Vertex_handle> newVerts;

        std::cout << "Iteration " << y << std::endl << "============================" << std::endl;
        vector<UpdateBundle> toProcess = pfC.getPath();

        for(int i = 0; i < toProcess.size(); ++i){

            std::cout << "Bundle: " << toProcess[i].handle->vertex()->point()
                        << ", " << toProcess[i].point
                        << ", " << toProcess[i].cost;
            std::cout << std::endl;

        }

        //process every second halfedge to divide triangle pairs
        for(vector<UpdateBundle>::iterator it = toProcess.begin(); it != toProcess.end(); ++it){

            PathPoly_3::Vertex_handle vh = adapt.split_on_edge((*it).handle, (*it).point);
            //PathPoly_3::Vertex_handle vh = adapt.find_halfedge_handle(*((*it).handle->vertex()), (*(*it).handle->opposite()->vertex()),(*it).point);
            vh->key = node_key((*it).cost, (*it).cost);
            std::cout << "Added vertex at " << vh->point() << " with cost " << vh->key << std::endl;
            char a;
            cin >> a;
            newVerts.push_front(vh);

            }

        //PathPoly_3::Vertex_handle vh = adapt.find_halfedge_handle(*(adapt.P.vertices_begin()), *(++(adapt.P.vertices_begin())), CGAL::Point_3<K>(0,0.5,0));

        pfC.handleChanges(newVerts);

    }
    pfC.getPath();

    //PathPoly_3::Vertex_handle vh = adapt.find_halfedge_handle(*(adapt.P.vertices_begin()), *(++(adapt.P.vertices_begin())), CGAL::Point_3<K>(0,0.5,0));

    std::cout << "Num. vertices: " << adapt.P.size_of_vertices() << std::endl;
    std::cout << "Num. Facets: " << adapt.P.size_of_facets() << std::endl;

	glutInit(&argc,argv);
	glutInitDisplayMode (GLUT_SINGLE | GLUT_RGBA);
	glutInitWindowSize(windowX,windowY);
	glutInitWindowPosition(0,0);
	glutCreateWindow("Adaptive Field D* - Visual Debugger");

	//glEnable( GL_POINT_SMOOTH );
    //glEnable( GL_BLEND );
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	//glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	//glEnable(GL_POINT_SPRITE);

	glutDisplayFunc(display);
	glutIdleFunc(display);

	glutKeyboardFunc(keyb);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseDrag);

	glutReshapeFunc(reshape);
	glutMainLoop();


}

