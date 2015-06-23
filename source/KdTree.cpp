/*
 * KdTree.cpp
 *
 *  Created on: 18 Jun 2015
 *      Author: ruben
 */

/**
 * Class KD
 */

#include "KdTree.h"
#include <float.h>
#include <utility>
#include <list>
#include <cmath>
#include "mesh.h"
#include "raytracing.h"
#include "aabbTriangle.h"

/**
 * KD::intersectsWithRay
 * Tests the bounding box (aabb) of this node against a ray
 * @param origin Origin of the ray
 * @param dest Destination of the rat
 * @param out distance Distance from the origin of the ray to the bounding box, if the ray intersects
 * @result True if the ray hits
 */

inline bool KD::interSectsWithRay(const Vec3Df & origin, const Vec3Df & dest, float & distance){
	//Translate the box with te origin of the ray
	Vec3Df lbfT = this->lbf - origin, rtrT = this->rtr - origin;
	Vec3Df hit1, hit2;
	Vec3Df ray = dest - origin;
	//Simple check: is the ray in the box?
	bool result = isBetween(origin, this->lbf, this->rtr);
	distance = 0;
	if(!result){
		//If the ray did not start in the box, does it intersect the box?
		result = boxTest(ray, lbfT, rtrT, hit1, hit2);
		distance = hit1.getLength();
	}

	return result;
}

/**
 * trianglesDistancePairComparison
 * Comparison function for two element of the type <distance, list>. Orders on distance
 * @result True if first is smaller than second
 */

bool trianglesDistancePairComparison(std::pair< float, std::vector<unsigned int> * > first,
		std::pair< float, std::vector<unsigned int> * > second){
	return first.first < second.first;
}

/**
 * Class KDLeaf
 */


/**
 * KDLeaf::KDLeaf
 * Constructs an empty leaf with aabb (lbf, rtr)
 * @param lbf LeftBottomFront coordinate of the aabb
 * @param rtr RightTopRead coordinate of the aabb
 */

KDLeaf::KDLeaf(Vec3Df lbf, Vec3Df rtr): KD(){
	this->lbf = lbf;
	this->rtr = rtr;
	this->triangles = std::vector<unsigned int>();
}

/**
 * KDLeaf::KDLeaf
 * Constructs a leaf without an aabb
 */

KDLeaf::KDLeaf(){
	KDLeaf(Vec3Df(), Vec3Df());
}

/**
 * isBetween
 * Checks if a point lies in the aabb defined by lbf en rtr
 * @param point Point to be tested
 * @param lfb LeftBottomFront of the aabb
 * @param rtr RightTopear of the aabb
 * @result True if the point is in the aabb
 */

inline bool isBetween(const Vec3Df & point, const Vec3Df & lbf, const Vec3Df & rtr){
	return point[0] >= lbf[0] && point[0] <= rtr[0] &&
			point[1] >= lbf[1] && point[1] <= rtr[1] &&
			point[2] >= lbf[2] && point[2] <= rtr[2];
}

/**
 * KDLeaf::add
 * Add a triangle to this leaf, if this triangle is inside the aabb of this leaf
 * @param triangle The index of the triangle in the mesh
 */

void KDLeaf::add(unsigned int triangle){
	Triangle t = MyMesh.triangles[triangle];
	Vec3Df h1, h2;
	Vec3Df p1 = MyMesh.vertices[t.v[0]].p, p2 = MyMesh.vertices[t.v[1]].p, p3 = MyMesh.vertices[t.v[2]].p;
	if(isBetween(p1, this->lbf, this->rtr) ||
			isBetween(p2, this->lbf, this->rtr) ||
			isBetween(p3, this->lbf, this->rtr)){
		this->triangles.push_back(triangle);
	}else{
		Vec3Df half = (this->rtr - this->lbf) / 2;
		Vec3Df middle = this->lbf + half;
		Vec3Df points [3] = {p1, p2, p3};
		if(triBoxOverlap(middle, half, points)){
			this->triangles.push_back(triangle);
		}
	}
}

/**
 * KDLeaf::optimizeBox
 * Shrinks the aabb to the smallest aabb possible with the triangles in this leaf
 */

void KDLeaf::optimizeBox(){
	if(this->triangles.size() > 0){
		float min1 = FLT_MAX, min2 = FLT_MAX, min3 = FLT_MAX, max1 = FLT_MAX * -1, max2 = FLT_MAX * -1, max3 = FLT_MAX * -1;
		for(std::vector<unsigned int>::iterator it = this->triangles.begin(); it != this->triangles.end(); ++it){
			unsigned int i = *it;
			for(int tv = 0; tv < 3; ++tv){

				Vertex v = MyMesh.vertices[MyMesh.triangles[i].v[tv]];
				if(v.p[0] > max1){
					max1 = v.p[0];
				}
				if(v.p[0] < min1){
					min1 = v.p[0];
				}
				if(v.p[1] > max2){
					max2 = v.p[1];
				}
				if(v.p[1] < min2){
					min2 = v.p[1];
				}
				if(v.p[2] > max3){
					max3 = v.p[2];
				}
				if(v.p[2] < min3){
					min3 = v.p[2];
				}
			}
		}
		if(min1 > this->lbf[0]){
			this->lbf[0] = min1;
		}
		if(min2 > this->lbf[1]){
			this->lbf[1] = min2;
		}
		if(min3 > this->lbf[2]){
			this->lbf[2] = min3;
		}
		if(max1 < this->rtr[0]){
			this->rtr[0] = max1;
		}
		if(max2 < this->rtr[1]){
			this->rtr[1] = max2;
		}
		if(max3 < this->rtr[2]){
			this->rtr[2] = max3;
		}
	}
}

/**
 * KDLeaf::cost
 * Calculates an estimate of the cost of shooting a ray through this leaf
 * @result Cost
 */

float KDLeaf::cost(){
	Vec3Df vol = this->rtr - this->lbf;
	if(this->triangles.size() > 0){
		float cost = vol[0]*vol[1]*vol[2] * 1000;
		if(cost < 0) cost *=-1;
		return cost + this->triangles.size();
	}
	return FLT_MAX;
}

/**
 * KDLeaf::getOrderedTriangles
 * Tests the bounding box (aabb) of this node against a ray, and return the triangles if the test succeeds
 * @param origin Origin of the ray
 * @param dest Destination of the ray
 * @param out list List with pairs of <distance, triangles>
 */

void KDLeaf::getOrderedTriangles(const Vec3Df & origin, const Vec3Df & dest,
		std::list< std::pair< float, std::vector<unsigned int> * > > & triangles){
	//triangles = std::list< std::pair< float, std::list<unsigned int> > >();
	float distance;
	if(this->interSectsWithRay(origin, dest, distance)){
		triangles.push_back(std::pair< float, std::vector<unsigned int> * >(distance, &this->triangles));
	}
}

/**
 * KDLeaf::prettyPrint
 * Print the contents of this leaf to stdout
 */

void KDLeaf::prettyPrint(){
	std::cout << "(LEAF: " << this->lbf << " " << this->rtr << " "
			<< this->triangles.size() << " " << this->cost() <<  ")" << std::endl;
}

/**
 * KDLeaf::prettyPrintHit
 * Tests the bounding box (aabb) of this leaf against a ray, and prints the result with some other properties
 */

void KDLeaf::prettyPrintHit(Vec3Df origin, Vec3Df dest){
	float ign;
	std::cout << "(LEAF: " << this->lbf << " " << this->rtr << " "
			<< this->triangles.size() << " " << this->cost() << " "
			<< this->interSectsWithRay(origin, dest, ign) <<  ")" << std::endl;
}

/**
 * class KDNode
 */

/**
 * KDNode::KDNode
 * Constructs a KDNode with an aabb and two leaves
 * @param lbf LeftBottomFront of the aabb
 * @param rtr RightTopRear of the aabb
 * @param left Left node or leaf beneath this node
 * @param right Right node or leaf beneath this node
 */

KDNode::KDNode(Vec3Df lbf, Vec3Df rtr, KD * left, KD * right): KD(){
	this->lbf = lbf;
	this->rtr = rtr;
	this->left = left;
	this->right = right;
}

/**
 * KDNode::build
 * Build a tree from one starting node. Stops if depth is zero, or if this node contains few triangles
 * @param from KDLeaf containing al the triangles and aabb to construct the tree from
 * @param depth The maximum depth that is allowed to beneath this node
 * @param parts Divide the aabb of from into 3 times parts-1 pairs of boxes to be tested
 * @result KDLeaf from if from is not divided, else a new node
 */

KD* KDNode::build(KDLeaf * from, const unsigned int depth, const unsigned int parts){
	if (depth == 0 || from->triangles.size() < 100) return from;
	float minCost = FLT_MAX;
	KDLeaf * minL, * minR;
	for(int axis = 0; axis < 3; ++axis){
		if(from->lbf[axis] == from->rtr[axis]) continue;//This is a plane!?
		Vec3Df step = ((from->rtr[axis] - from->lbf[axis])/parts) * Vec3Df(axis == 0, axis == 1, axis == 2);
		for(unsigned int i = 1; i < parts; ++ i){
			KDLeaf * left = new KDLeaf(from->lbf, from->rtr - (parts - i) * step);
			KDLeaf * right = new KDLeaf(from->lbf + i * step, from->rtr);
			for(std::vector<unsigned int>::iterator t = from->triangles.begin();
					t != from->triangles.end(); ++t){
				left->add(*t);
				right->add(*t);
			}
			float cost = left->cost() + right->cost();
			if(cost < minCost){
				minCost = cost;
				minL = left;
				minR = right;
			}
		}
	}
	if(minCost == FLT_MAX) return from;
	minL->optimizeBox();
	minR->optimizeBox();
	return new KDNode(from->lbf, from->rtr, KDNode::build(minL, depth - 1, parts), KDNode::build(minR, depth - 1, parts));
}

/**
 * KDNode::getOrderedTriangles
 * Tests the bounding box (aabb) of this node against a ray, and return the triangles of the leaves if the test succeeds
 * @param origin Origin of the ray
 * @param dest Destination of the ray
 * @param out list List with pairs of <distance, triangles>
 */

void KDNode::getOrderedTriangles(const Vec3Df & origin, const Vec3Df & dest,
		std::list< std::pair< float, std::vector<unsigned int> * > > & triangles){
	std::list< std::pair< float, std::vector<unsigned int> * > > fromLeft;
	float distance;
	if(this->interSectsWithRay(origin, dest, distance)){
		this->left->getOrderedTriangles(origin, dest, fromLeft);
		std::list< std::pair< float, std::vector<unsigned int> * > > fromRight;
		this->right->getOrderedTriangles(origin, dest, fromRight);
		if(fromRight.size() > 0 && fromLeft.size() > 0){
			fromLeft.merge(fromRight, trianglesDistancePairComparison);
		}else if(fromRight.size() > 0){
			fromLeft = fromRight;
		}
	}
	triangles = fromLeft;
}

/**
 * KDNode::prettyPrint
 * Print the contents of this node and its leaves to stdout
 */

void KDNode::prettyPrint(){
	std::cout << "(NODE: " << this->lbf << " " << this->rtr << std::endl;
	std::cout << "LEFT: ";
	this->left->prettyPrint();
	std::cout << "RIGHT: ";
	this->right->prettyPrint();
	std::cout << ")" << std::endl;
}

/**
 * KDNode::prettyPrintHit
 * Tests the bounding box (aabb) of this node against a ray, and prints the result with some other properties
 */

void KDNode::prettyPrintHit(Vec3Df origin, Vec3Df dest){
	float ign;
	std::cout << "(NODE: " << this->lbf << " " << this->rtr << " " << this->interSectsWithRay(origin, dest, ign) << std::endl;
	std::cout << "LEFT: ";
	this->left->prettyPrintHit(origin, dest);
	std::cout << "RIGHT: ";
	this->right->prettyPrintHit(origin, dest);
	std::cout << ")" << std::endl;
}
