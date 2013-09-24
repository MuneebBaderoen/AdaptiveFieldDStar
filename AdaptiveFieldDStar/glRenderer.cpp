#define _USE_MATH_DEFINES
#include "GL/glew.h"
#include <GL/GL.h>
#include <GL/glut.h>


#include "meshAdaptor.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

MeshAdaptor m; 
int currentFace = 0;
int windowX = 800, windowY = 600;



bool cameraTrue = true;

Vector3 modelMin, modelMax;

Vector3 modelPos = Vector3(0,0,0);

//float cx = 0.0f, cy = 0.0f, cz = 0.0f, 
float crx = 120.0f, cry = 60.0f, zoom = 50;

Vector3 camPos;
Vector3 camUp = Vector3(0,1,0);

float inc = 0.1f;
using namespace std;

int mousePosX, mousePosY;
int oldMousePosX, oldMousePosY;
bool mouseClicked = false;
bool interactiveMode = false;
/*

string getText(){
	
	string out ="";
	out+="Zoom: ";
	char num [20];
	out+=itoa(zoom, num,10);
	out+="   Cry: ";
	out+=itoa(cry, num,10);
	out+="   Crx: ";
	out+=itoa((int)(crx<0?((int)crx%360+360):crx)%360, num,10);
	out+="   Pixel Cutoff: ";
	out+=itoa(pixelCutoff, num, 10);
	out+=".";
	int pixelCutoffDec = int(pixelCutoff*10)%10;
	out+=itoa(pixelCutoffDec, num, 10);

	out+="   Minimum Cutoff: ";
	out+=itoa(minimumPixelCutOff, num, 10);
	out+=".";
	int minimumPixelCutOffDec = int(minimumPixelCutOff*10)%10;
	out+=itoa(minimumPixelCutOffDec, num, 10);
	
	


	return out;
}
string getText2(){
	
	string out ="";	
	char num [20];
	out+="Currently Rendering: ";
	out+=itoa(currentlyRendering, num, 10);
	out+="/";
	out+=itoa(parser.numVerts, num, 10);
	out+="  FPS: ";
	
	
	out+=itoa(frameRate, num, 10);	
	out+="  Desired FPS: ";
	out+=itoa(desiredFrameRate, num, 10);
	out+="  Freeze Frustum: ";
	out+=frustumFollow?"false":"true";
	return out;
}

void matchFrameRate(){
	float cutoffDelta = 0;

	if((desiredFrameRate-frameRate)>10)
		cutoffDelta+=10;
	else if((desiredFrameRate-frameRate)>5)
		cutoffDelta+=0.2f;
	else if((frameRate-desiredFrameRate)>10)
		cutoffDelta-=1.0f;
	else if((frameRate-desiredFrameRate)>15)
		cutoffDelta-=2.0f;
	else
		interactivePixelCutoff=(float)pixelCutoff;
	if(pixelCutoff+cutoffDelta>minimumPixelCutOff)
		pixelCutoff+=cutoffDelta;
}
*/

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
void keyb(unsigned char key, int x, int y){


	if(key=='s'){
		m.split_triangles();
	}
	if(key=='r'){
		resetCam();
	}	
	if(key=='6'){
		currentFace++;
		currentFace%=m.getSurface().size_of_facets();
	}
	if(key=='4'){
		currentFace--;	
		currentFace+=m.getSurface().size_of_facets();
		currentFace%=m.getSurface().size_of_facets();
	}
	if(key=='-'){
		zoom+=0.5f;
	}
	if(key=='+'){
		zoom-=0.5f;	
	}
}

void reshape(int width, int height)
{
	
	glViewport(0,0,(GLsizei) width,(GLsizei) height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat) width/(GLfloat) height,1.0, 100);

	glMatrixMode(GL_MODELVIEW);
}
void Text(const char * text, int length, int x, int y)
{
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
	
	Polyhedron P = m.getSurface();
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
	
	//std::string filename = "Resources/bunny_recon/bun_zipper.ply";
	std::string filename = "Resources/tetra.ply";
	m.init((char *) filename.c_str());
	
	//Polyhedron P;
	

	
	//trianglesArray=parser.readPlyFile(fName, 20.0f);

	//modelMin = parser.vertices[0];
	//modelMax = parser.vertices[0];
	/*
	for(std::vector<Vertex>::iterator iter = parser.vertices.begin(); iter!=parser.vertices.end();++iter){
		modelMin=modelMin.minVec((*iter));
		modelMax=modelMax.maxVec((*iter));
	}

	modelPos=Vector3((modelMin.x+modelMax.x)/2,
		(modelMin.y+modelMax.y)/2,
		(modelMin.z+modelMax.z)/2);

	cout<<"Model Min: ("<<modelMin.x<<", "<<modelMin.y<<", "<<modelMin.z<<")"<<endl;
	cout<<"Model Max: ("<<modelMax.x<<", "<<modelMax.y<<", "<<modelMax.z<<")"<<endl;
	cout<<"Model Pos: ("<<modelPos.x<<", "<<modelPos.y<<", "<<modelPos.z<<")"<<endl;
	*/

	camPos=Vector3(0,0,0-zoom);

	

	

	
	
	

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

