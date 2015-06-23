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

bool KD::interSectsWithRay(const Vec3Df & origin, const Vec3Df & dest, float & distance){
	Vec3Df lbfT = this->lbf - origin, rtrT = this->rtr - origin;
	Vec3Df hit1, hit2;
	Vec3Df ray = dest - origin;
	bool result = isBetween(origin, this->lbf, this->rtr);
	distance = 0;
	if(!result){
		result = boxTest(ray, lbfT, rtrT, hit1, hit2);
		distance = hit1.getLength();
	}

	return result;
}

bool trianglesDistancePairComparison(std::pair< float, std::vector<unsigned int> * > first,
		std::pair< float, std::vector<unsigned int> * > second){
	return first.first < second.first;
}

/**
 * Class KDLeaf
 */


KDLeaf::KDLeaf(Vec3Df lbf, Vec3Df rtr): KD(){
	this->lbf = lbf;
	this->rtr = rtr;
	this->triangles = std::vector<unsigned int>();
}

KDLeaf::KDLeaf(){
	KDLeaf(Vec3Df(), Vec3Df());
}

inline bool isBetween(const Vec3Df & point, const Vec3Df & lbf, const Vec3Df & rtr){
	return point[0] >= lbf[0] && point[0] <= rtr[0] &&
			point[1] >= lbf[1] && point[1] <= rtr[1] &&
			point[2] >= lbf[2] && point[2] <= rtr[2];
}

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

float KDLeaf::cost(){
	Vec3Df vol = this->rtr - this->lbf;
	if(this->triangles.size() > 0){
		float cost = vol[0]*vol[1]*vol[2] * 1000;
		if(cost < 0) cost *=-1;
		return cost + this->triangles.size();
	}
	return FLT_MAX;
}

void KDLeaf::getOrderedTriangles(const Vec3Df & origin, const Vec3Df & dest,
		std::list< std::pair< float, std::vector<unsigned int> * > > & triangles){
	//triangles = std::list< std::pair< float, std::list<unsigned int> > >();
	float distance;
	if(this->interSectsWithRay(origin, dest, distance)){
		triangles.push_back(std::pair< float, std::vector<unsigned int> * >(distance, &this->triangles));
	}
}

void KDLeaf::prettyPrint(){
	std::cout << "(LEAF: " << this->lbf << " " << this->rtr << " "
			<< this->triangles.size() << " " << this->cost() <<  ")" << std::endl;
}

void KDLeaf::prettyPrintHit(Vec3Df origin, Vec3Df dest){
	float ign;
	std::cout << "(LEAF: " << this->lbf << " " << this->rtr << " "
			<< this->triangles.size() << " " << this->cost() << " "
			<< this->interSectsWithRay(origin, dest, ign) <<  ")" << std::endl;
}

/**
 * class KDNode
 */

KDNode::KDNode(Vec3Df lbf, Vec3Df rtr, KD * left, KD * right): KD(){
	this->lbf = lbf;
	this->rtr = rtr;
	this->left = left;
	this->right = right;
}

KD* KDNode::build(KDLeaf * from, const unsigned int depth){
	if (depth == 0 || from->triangles.size() < 100) return from;
	int parts = 4;
	float minCost = FLT_MAX;
	KDLeaf * minL, * minR;
	for(int axis = 0; axis < 3; ++axis){
		if(from->lbf[axis] == from->rtr[axis]) continue;//This is a plane!?
		Vec3Df step = ((from->rtr[axis] - from->lbf[axis])/parts) * Vec3Df(axis == 0, axis == 1, axis == 2);
		for(int i = 1; i < parts; ++ i){
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
	return new KDNode(from->lbf, from->rtr, KDNode::build(minL, depth - 1), KDNode::build(minR, depth - 1));
}

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

void KDNode::prettyPrint(){
	std::cout << "(NODE: " << this->lbf << " " << this->rtr << std::endl;
	std::cout << "LEFT: ";
	this->left->prettyPrint();
	std::cout << "RIGHT: ";
	this->right->prettyPrint();
	std::cout << ")" << std::endl;
}
void KDNode::prettyPrintHit(Vec3Df origin, Vec3Df dest){
	float ign;
	std::cout << "(NODE: " << this->lbf << " " << this->rtr << " " << this->interSectsWithRay(origin, dest, ign) << std::endl;
	std::cout << "LEFT: ";
	this->left->prettyPrintHit(origin, dest);
	std::cout << "RIGHT: ";
	this->right->prettyPrintHit(origin, dest);
	std::cout << ")" << std::endl;
}
