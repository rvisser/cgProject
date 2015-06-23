#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "raytracing.h"
#include <omp.h>
#include <float.h>
#include "KdTree.h"

#define dot(u,v)	Vec3Df::dotProduct(u, v)

//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;
float lightstrength = 1.0f;
float ambientstrenght = 0.5f;
KD * tree;
Vec3Df originColor = Vec3Df(0,1,1);
Vec3Df destinationColor = Vec3Df(0,0,1);

//use this function for any preprocessing of the mesh.
void init() {
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they 
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj", 
	//otherwise the application will not load properly
	//OR make sure the .obj is located in the working directory
	//MyMesh.loadMesh("cube.obj", true);
	MyMesh.loadMesh("dodgeColorTest.obj", true);
	MyMesh.computeVertexNormals();


	Vec3Df lbf = Vec3Df(FLT_MAX * -1,FLT_MAX * -1, FLT_MAX * -1), rtr = Vec3Df(FLT_MAX, FLT_MAX, FLT_MAX);

	KDLeaf * start = new KDLeaf(lbf, rtr);

	for(unsigned int i = 0; i < MyMesh.triangles.size(); ++i){
		start->add(i);
	}
	start->optimizeBox();

	start->prettyPrint();

	tree = KDNode::build(start, 5);

	tree->prettyPrint();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
	MyLightPositions.push_back(Vec3Df(0.5, 0.0, -0.5));
}

/*
 * given 3 points v1, v2 and v3 returns the normal of the plane spanned by these points
 */
inline Vec3Df getNormal(const Vec3Df & v1, const Vec3Df & v2, const Vec3Df & v3) {
	Vec3Df e1 = v1 - v3;
	Vec3Df e2 = v2 - v3;

	//Calculate the normal on the plane spanned by the edges
	Vec3Df n = Vec3Df::crossProduct(e1, e2);
	n.normalize();
	return n;
}

/*
 * given a ray returns the delta at which the ray passes through the plane
 * defined by point v1 and normal n.
 */
inline float PlaneTest(Vec3Df ray, Vec3Df n, Vec3Df v1) {
	//Distance from origin to the plane
	float dist = Vec3Df::dotProduct(v1, n);

	//Calculate the hit parameter of the ray, and the point in (or next to) the triangle where the ray hits
	return dist / Vec3Df::dotProduct(ray, n);
}

/*
 * checks if the point p lies within a triangle with corners v1, v2 and v3.
 */
inline bool TriangleTest(Vec3Df p, Vec3Df a, Vec3Df b, Vec3Df c) {
	float u, v, w;
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
	return (u >= 0 && u <= 1 && v >= 0 && u + v <= 1);
}

inline float min(float f1, float f2){
	if (f1 < f2) return f1;
	return f2;
}

inline float max(float f1, float f2){
	if (f1 > f2) return f1;
	return f2;
}

/*
 * ray = The ray to be tested
 * p1 = Low Left Front (0,0,0)
 * p2 = High Right Back (1,1,1)
 * tIn = entrypoint
 * tOut = exitpoint
 */
bool boxTest(Vec3Df ray, Vec3Df p1, Vec3Df p2, Vec3Df &tIn, Vec3Df &tOut) {
	float
		txMin = p1[0] / ray[0],
		txMax = p2[0] / ray[0],
		tyMin = p1[1] / ray[1],
		tyMax = p2[1] / ray[1],
		tzMin = p1[2] / ray[2],
		tzMax = p2[2] / ray[2];
	tIn = Vec3Df(min(txMin, txMax), min(tyMin, tyMax), min(tzMin, tzMax));
	tOut = Vec3Df(max(txMin, txMax), max(tyMin, tyMax), max(tzMin, tzMax));
	float inPar = max(max(tIn[0], tIn[1]), tIn[2]);
	float outPar = min(min(tOut[0], tOut[1]), tOut[2]);
	return inPar <= outPar && outPar >= 0;
}

/*
 * Given a ray (origin -> dest), calculates the first hit point to a triangle at index triangle in the mesh.
 * Sets triangle to -1 if no intersection with a triangle and this ray is found
 */
inline void castRay(const Vec3Df & origin, const Vec3Df & dest, int & triangle, Vec3Df & point, Vec3Df & normal) {
	//Initialize the minimum distance at quite a large value
	float nearest = FLT_MAX;
	triangle = -1;
	std::list< std::pair< float, std::list<unsigned int> > > list = std::list< std::pair< float, std::list<unsigned int> > >();
	tree->getOrderedTrianlges(origin, dest, list);
	bool found = false;
	for(std::list< std::pair< float, std::list<unsigned int> > >::iterator it1 = list.begin(); it1 != list.end() && !found; ++it1){
		for(std::list<unsigned int>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); ++ it2){
			unsigned int i  = *it2;
	//for (unsigned int i = 0; i < MyMesh.triangles.size(); ++i) {
		//Get all vertices and calculate edges, translated to the origin of the ray as new origin
		Vec3Df v1 = MyMesh.vertices[MyMesh.triangles[i].v[0]].p - origin;
		Vec3Df v2 = MyMesh.vertices[MyMesh.triangles[i].v[1]].p - origin;
		Vec3Df v3 = MyMesh.vertices[MyMesh.triangles[i].v[2]].p - origin;
		Vec3Df ray = dest - origin;
		Vec3Df norm = getNormal(v1, v2, v3);
		float hit = PlaneTest(ray, norm, v1);
		if (hit > 0 && hit < nearest) {
			//Make sure that p is inside the triangle using barycentric coordinates
			if (TriangleTest(hit * ray, v1, v2, v3)) {
				found = true;
				point = hit * ray;
				nearest = hit;
				triangle = i;
				normal = norm;
			}
		}
		}
	}
}

/*
 * given a rat (origin -> dest), test if the ray intersects any triangles.
 * this function is faster and should be used for line of sight tests.
 */
inline bool testRay(const Vec3Df & origin, const Vec3Df & dest) {
	std::list< std::pair< float, std::list<unsigned int> > > list = std::list< std::pair< float, std::list<unsigned int> > >();
	tree->getOrderedTrianlges(origin, dest, list);
	for(std::list< std::pair< float, std::list<unsigned int> > >::iterator it1 = list.begin(); it1 != list.end(); ++it1){
		for(std::list<unsigned int>::iterator it2 = it1->second.begin(); it2 != it1->second.end(); ++ it2){
			unsigned int i  = *it2;
	//for (unsigned int i = 0; i < MyMesh.triangles.size(); ++i) {
		//Get all vertices and calculate edges, translated to the origin of the ray as new origin
		Vec3Df v1 = MyMesh.vertices[MyMesh.triangles[i].v[0]].p - origin;
		Vec3Df v2 = MyMesh.vertices[MyMesh.triangles[i].v[1]].p - origin;
		Vec3Df v3 = MyMesh.vertices[MyMesh.triangles[i].v[2]].p - origin;
		Vec3Df ray = dest - origin;
		float dist = ray.getLength();
		Vec3Df norm = getNormal(v1, v2, v3);
		float hit = PlaneTest(ray, norm, v1);
		if (hit > 0) {
			//Make sure that p is inside the triangle using barycentric coordinates
			if (TriangleTest(hit * ray, v1, v2, v3)) {
				if (hit * ray.getLength()<dist) return true;
			}
		}
	}
	}
	return false;
}

//Calculate the actual diffuse color, given the diffuse color of the material and a ray hit point p
inline Vec3Df calcDiffuse(const Vec3Df & objectColor, const Vec3Df & p, const Vec3Df & normal) {
	//TODO: check of any of this is correct
	Vec3Df result = Vec3Df(0, 0, 0);
	for (std::vector<Vec3Df>::iterator l = MyLightPositions.begin();
			l != MyLightPositions.end(); ++l) {
		//Translate point p back to world coordinates!
		//Not sure if I should, this seems to work
		Vec3Df at, norm;
		if (!testRay(p, *l)) {
			//No intersection :)
			Vec3Df lVector = Vec3Df(l->p) - p;
			lVector.normalize();
			/*
			 * calculates the strength of the light with inverse-square falloff
			 * at distance 0 the light intensity is 1.0 I
			 * at distance 1 the light intensity is 1/4 I
			 * at distance 2 the light intensity is 1/9 I
			 */
			Vec3Df lightDistance = (*l - p);
			float lightDiffuse = lightstrength / pow(lightDistance.getLength()+1,2);
			result += dot(lVector, normal)* objectColor * lightDiffuse;
		}
	}
	return result;
}

/*
 * adds the ambient light factor to obtain the ambient diffuse color
 */
inline Vec3Df calcAmbient(const Vec3Df & objectColor, const Vec3Df & p, const Vec3Df & normal) {
	return objectColor * ambientstrenght;
}

/*
 * given a ray, point and normal reflects the ray and gives the result back as a reflecting color.
 */
inline Vec3Df calcReflect(const Vec3Df & objectColor, const Vec3Df & p, Vec3Df ray, const Vec3Df & normal, unsigned int bounces) {
	ray.normalize();
	return objectColor*performRayTracing(p,p + ray - 2*dot(normal,ray)*normal, bounces);
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest, unsigned int bounces) {
	//Default color: black background
	Vec3Df color = Vec3Df(0, 0, 0);
	if (!bounces) return color;
	int triangle;
	Vec3Df p, norm;
	castRay(origin, dest, triangle, p, norm);
	if (triangle >= 0) {
		unsigned int triMat = MyMesh.triangleMaterials.at(triangle);
		Material m = MyMesh.materials.at(triMat);
		Vec3Df diffuse = m.Kd();
		Vec3Df ambient = m.Ka();
		Vec3Df specular = m.Ks();
		//Translate point p back to world coordinates!
		color += calcDiffuse(diffuse, p * 0.9999 + origin, norm); // if calcDiffuse is working
		color += calcAmbient(diffuse, p, norm);
		color += Vec3Df(0,0,0);//calcSpecular;
		/*if (diffuse[1]>0.8,diffuse[1]>0.8,diffuse[2]>0.8) {
			color += calcReflect(Vec3Df(1,1,1),p * 0.9999 + origin,dest-origin,norm,--bounces);
		}*///uncomment this to make white surfaces reflective
	}

	return color;
}

void yourDebugDraw() {
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();

	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1, 1, 1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (unsigned int i = 0; i < MyLightPositions.size(); ++i) {
		if (i == selectedLight) {
			glColor3f(1, 0, 0);
		}
		glVertex3fv(MyLightPositions[i].pointer());
		if (i == selectedLight) {
			glColor3f(1, 1, 1);
		}
	}
	glEnd();
	glPopAttrib(); //restore all GL attributes
	//The Attrib commands maintain the state. 
	//e.g., even though inside the two calls, we set
	//the color to white, it will be reset to the previous 
	//state after the pop.

	//as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(originColor[0],originColor[1],originColor[3]);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(destinationColor[0],destinationColor[1],destinationColor[2]);
	glVertex3f(testRayDestination[0], testRayDestination[1],
			testRayDestination[2]);
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
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin,
		const Vec3Df & rayDestination) {

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin = rayOrigin;
	testRayDestination = rayDestination;

	// do here, whatever you want with the keyboard input t.

	//...

	std::cout << t << " pressed! The mouse was in location " << x << "," << y
			<< "!" << std::endl;
	switch (t)
	{
		case 's':{
			//Trace single ray
			kdTreeVerbose = true;
			int triangle;
			Vec3Df p, n;
			castRay(testRayOrigin, testRayDestination, triangle, p, n);
			destinationColor = performRayTracing(testRayOrigin,testRayDestination,1);
			originColor = destinationColor;
			testRayDestination = p + testRayOrigin;
			kdTreeVerbose = false;
			break;
		}
	}
}
