#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "raytracing.h"
#include <omp.h>
#include <float.h>

#define dot(u,v)	Vec3Df::dotProduct(u, v)

//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
Vec3Df testRayOrigin;
Vec3Df testRayDestination;


//use this function for any preprocessing of the mesh.
void init()
{
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they 
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj", 
	//otherwise the application will not load properly
	//OR make sure the .obj is located in the working directory
    MyMesh.loadMesh("cube.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

inline void Barycentric(Vec3Df p, Vec3Df a, Vec3Df b, Vec3Df c, float &u, float &v, float &w)
{
	Vec3Df v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = Vec3Df::dotProduct(v0, v0);
    float d01 = Vec3Df::dotProduct(v0, v1);
    float d11 = Vec3Df::dotProduct(v1, v1);
    float d20 = Vec3Df::dotProduct(v2, v0);
    float d21 = Vec3Df::dotProduct(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

/*
 * Given a ray (origin -> dest), calculate a possible hit point point in the triangle at index triangle in the mesh.
 * Sets triangle to -1 if no intersection with a triangle and this ray is found
 */
void getTriangleIntersection(const Vec3Df & origin, const Vec3Df & dest, int & triangle, Vec3Df & point){
	//Initialise the minimum distance at quite a large value
	float nearest = FLT_MAX;
	triangle = -1;
	for (unsigned int i=0;i<MyMesh.triangles.size();++i)
	{
		//Get all vertices and calculate edges, translated to the origin of the ray as new origin
		Vec3Df v1 = MyMesh.vertices[MyMesh.triangles[i].v[0]].p - origin;
		Vec3Df v2 = MyMesh.vertices[MyMesh.triangles[i].v[1]].p - origin;
		Vec3Df v3 = MyMesh.vertices[MyMesh.triangles[i].v[2]].p - origin;

		Vec3Df e1 = v1 - v3;
		Vec3Df e2 = v2 - v3;

		Vec3Df ray = dest - origin;

		//Calculate the normal on the plane spanned by the edges
		Vec3Df n = Vec3Df::crossProduct(e1, e2);
		n.normalize();

		//Distance from origin to the plane
		float D = Vec3Df::dotProduct(v1, n);

		//Calculate the hit parameter of the ray, and the point in (or next to) the triangle where the ray hits
		float hit = D / Vec3Df::dotProduct(ray, n);
		Vec3Df p = hit * ray;

		if(hit > 0 && hit < nearest){
			//Make sure that p is inside the triangle using barycentric coordinates
			float a, b, ab;
			Barycentric(p, v1, v2, v3, a, b, ab);
			if(a>=0 && a <= 1 && b>=0 && a + b <= 1)
			{
				point = p;
				nearest = hit;
				triangle = i;

			}
		}

	}

}

//Calculate the actual diffuse colour, given the diffuse colour of the material and a ray hit point p
Vec3Df calcDiffuse(const Vec3Df & colour, const Vec3Df & p){
	//TODO: fix this function, or re-implement a better working variant...
	Vec3Df result = Vec3Df(0,0,0);
	for(std::vector<Vec3Df>::iterator l = MyLightPositions.begin(); l != MyLightPositions.end(); ++l){

		Vec3Df at;
		int intersection;
		getTriangleIntersection(p, *l, intersection, at);
		if(intersection < 0){
			//No intersection :)
			result += colour;
		}

	}
	return result;
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	//Default colour: black background
	Vec3Df colour = Vec3Df(0,0,0);
	int triangle;
	Vec3Df p;
	getTriangleIntersection(origin, dest, triangle, p);
	if(triangle >= 0){
		unsigned int triMat = MyMesh.triangleMaterials.at(triangle);
		Material m = MyMesh.materials.at(triMat);
		Vec3Df diffuse = m.Kd();
		Vec3Df ambient = m.Ka();
		Vec3Df specular = m.Ks();
		//Translate point p back to world coordinates!
		colour += calcDiffuse(diffuse, p * 0.9999 + origin);// if calcDiffuse is working

	}

	return colour;
}



void yourDebugDraw()
{
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();
	
	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (unsigned int i=0;i<MyLightPositions.size();++i){
		if(i == selectedLight){
			glColor3f(1,0,0);
		}
		glVertex3fv(MyLightPositions[i].pointer());
		if(i == selectedLight){
			glColor3f(1,1,1);
		}
	}
	glEnd();
	glPopAttrib();//restore all GL attributes
	//The Attrib commands maintain the state. 
	//e.g., even though inside the two calls, we set
	//the color to white, it will be reset to the previous 
	//state after the pop.


	//as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0,1,1);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(0,0,1);
	glVertex3f(testRayDestination[0], testRayDestination[1], testRayDestination[2]);
	glEnd();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3fv(MyLightPositions[0].pointer());
	glEnd();
	glPopAttrib();
	
	//draw whatever else you want...
	////glutSolidSphere(1,10,10);
	////allows you to draw a sphere at the origin.
	////using a glTranslate, it can be shifted to whereever you want
	////if you produce a sphere renderer, this 
	////triangulated sphere is nice for the preview
}


//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved: 
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current 
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering. 
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray. 
//    It then stores the result in an image "result.ppm".
//    Initially, this function is fast (performRaytracing simply returns 
//    the target of the ray - see the code above), but once you replaced 
//    this function and raytracing is in place, it might take a 
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin=rayOrigin;	
	testRayDestination=rayDestination;
	
	// do here, whatever you want with the keyboard input t.
	
	//...
	
	
	std::cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<std::endl;	
}
