#ifndef HELPER_CLASSES_H
#define HELPER_CLASSES_H

#include <iostream>

class Vector3{	
public:
	float x, y, z;

	Vector3(){x=0, y=0, z=0;}
	Vector3(float x1, float y1, float z1){
		x=x1;
		y=y1;
		z=z1;
	}
	Vector3(Vector3& vec){
		x=vec.x;
		y=vec.y;
		z=vec.z;
	}
	
	Vector3 rotate(float angle, Vector3& vec){
		using namespace std;
		float c = cos(angle);
		float s = sin(angle);
		float t = 1.0 - c;

		cout<<c<<" "<<s<<" "<<t<<endl;

		float m[3][3] = {
			{ (t*vec.x*vec.x + c),			(t*vec.y*vec.x - s*vec.z),	(t*vec.z*vec.x + s*vec.y) },
			{ (t*vec.x*vec.y + s*vec.z),	(t*vec.y*vec.y + c),		(t*vec.z*vec.y - s*vec.x) },
			{ (t*vec.x*vec.z - s*vec.y),	(t*vec.y*vec.z + s*vec.x),	(t*vec.z*vec.z + c)			}
		};


		Vector3 out;

		for(int i = 0; i<3;++i)
			for(int j = 0; j<3;++j)		
				out[i]+=m[i][j]*vec[j];
		
		
		return out;

	}

	float& operator[](int index){
		if(index==0)
			return x;
		if(index==1)
			return y;
		if(index==2)
			return z;
		float out = 0;
		return out;
	}

	void operator+=(Vector3& vec){
		x+=vec.x;
		y+=vec.y;
		z+=vec.z;		
	}
	void operator-=(Vector3& vec){
		x-=vec.x;
		y-=vec.y;
		z-=vec.z;		
	}
	void operator*=(float num){
		x*=num;
		y*=num;
		z*=num;		
	}
	void operator/=(float num){
		x/=num;
		y/=num;
		z/=num;		
	}

	Vector3 operator+(Vector3& vec){
		return Vector3(x+vec.x, y+vec.y, z+vec.z);		
	}
	Vector3 operator-(Vector3& vec){
		return Vector3(x-vec.x, y-vec.y, z-vec.z);		
	}
	Vector3 operator*(float num){
		return Vector3(x*num, y*num, z*num);		
	}
	Vector3 operator/(float num){
		return Vector3(x/num, y/num, z/num);		
	}

	Vector3 minVec(Vector3& vec){	
		using namespace std;
		return Vector3(min<float>(x,vec.x),min<float>(y,vec.y),min<float>(z,vec.z));
	}
	Vector3 maxVec(Vector3& vec){
		using namespace std;
		return Vector3(max<float>(x,vec.x),max<float>(y,vec.y),max<float>(z,vec.z));
	}

	float length(){
		return sqrtf(x * x + y * y + z * z);
	}	
	float length2(){
		return x * x + y * y + z * z;
	}
	Vector3 normalize(){
		float magnitude = length();
		x /= magnitude;
		y /= magnitude;
		z /= magnitude;
		return Vector3(x,y,z);
	}

	float dotVector3D(const Vector3& vec) const
	{
		return x * vec.x + y * vec.y + z * vec.z;
	}
	Vector3 cross(const Vector3& vec){
		
		return Vector3(	(y*vec.z)-(vec.y*z),
						-(x*vec.z)+(vec.x*z),
						(x*vec.y)-(y*vec.x)
					);
			
	}

	friend std::ostream& operator<<(std::ostream &strm, const Vector3 &a) {
		return strm << "Vector3(" << a.x<<", "<<a.y<<", "<<a.z << ")";
	}
};

class Vertex: public Vector3{
	public:
	Vector3 normal;
	float longestEdge;

	Vertex(){
	longestEdge=0;
	}
	Vertex(float v1, float v2, float v3){
		x=v1;
		y=v2;
		z=v3;
		longestEdge=0;
	}

	Vertex(Vector3& vec){
		x=vec.x;
		y=vec.y;
		z=vec.z;
		longestEdge=0;
	}

	Vertex(const Vector3& vec){
		x=vec.x;
		y=vec.y;
		z=vec.z;
		longestEdge=0;
	}

	friend std::ostream& operator<<(std::ostream &strm, const Vertex &a) {
		return strm << "Vertex(" << a.x<<", "<<a.y<<", "<<a.z << ")";
	}

	void setNormal(const Vector3& nor){
		normal = nor;
	}


	

	
};


class Triangle{
public:
	Vertex v1,v2,v3;
	Vector3 normal;
	unsigned char nverts;    /* number of vertex indices in list */
	int *verts;              /* vertex index list */

	Triangle(){}	
	Triangle(const Triangle& tri){
		v1=tri.v1;
		v2=tri.v2;
		v3=tri.v3;
	}
	Triangle(Vertex& vt1, Vertex& vt2, Vertex& vt3){
		v1=Vertex(vt1);
		v2=Vertex(vt2);
		v3=Vertex(vt3);
	}	
	Triangle(Vector3& vt1, Vector3& vt2, Vector3& vt3){
		v1=Vertex(vt1);
		v2=Vertex(vt2);
		v3=Vertex(vt3);
	}

	void calculateNormal(void){
		using namespace std;
		Vector3 temp1=v1-v2;
		Vector3 temp2=v1-v3;
		normal = temp2.cross(temp1);
		//cout<<temp1.x<<" "<<temp1.y<<" "<<temp1.z<<endl;
		//cout<<temp2.x<<" "<<temp2.y<<" "<<temp2.z<<endl;
	}
	void calculateLongestEdge(){
		float temp = (v1-v2).length();		
		if(temp>v1.longestEdge)
			v1.longestEdge=(float)temp;
		if(temp>v2.longestEdge)
			v2.longestEdge=(float)temp;

		temp = (v3-v2).length();
		if(temp>v2.longestEdge)
			v2.longestEdge=(float)temp;
		if(temp>v3.longestEdge)
			v3.longestEdge=(float)temp;

		temp = (v1-v3).length();
		if(temp>v1.longestEdge)
			v1.longestEdge=(float)temp;
		if(temp>v3.longestEdge)
			v3.longestEdge=(float)temp;		
	}



};
 


#endif