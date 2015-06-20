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
#include "mesh.h"

bool KD::interSectsWithRay(Vec3Df & origin, Vec3Df & dest, float & distance){
	distance = 1;
	return true; //Add box detection here.
}

bool trianglesDistancePairComparison(std::pair< float, std::list<unsigned int> > first,
		std::pair< float, std::list<unsigned int> > second){
	return first.first < second.first;
}

/**
 * Class KDLeaf
 */


KDLeaf::KDLeaf(Vec3Df lbf, Vec3Df rtr): KD(){
	this->lbf = lbf;
	this->rtr = rtr;
	this->triangles = std::list<unsigned int>();
}

KDLeaf::KDLeaf(){
	KDLeaf(Vec3Df(), Vec3Df());
}

bool isBetween(Vec3Df point, Vec3Df lbf, Vec3Df rtr){
	return point[0] > lbf[0] && point[0] < rtr[0] &&
			point[1] > lbf[1] && point[1] < rtr[1] &&
			point[2] > lbf[2] && point[2] < rtr[2];
}

void KDLeaf::add(unsigned int triangle){
	Triangle t = MyMesh.triangles[triangle];
	Vec3Df p1 = MyMesh.vertices[t.v[0]].p, p2 = MyMesh.vertices[t.v[1]].p, p3 = MyMesh.vertices[t.v[2]].p;
	if(isBetween(p1, this->lbf, this->rtr) ||
			isBetween(p2, this->lbf, this->rtr) ||
			isBetween(p3, this->lbf, this->rtr)){
		this->triangles.push_front(triangle);
	}
}
float KDLeaf::cost(){
	Vec3Df vol = this->rtr - this->lbf;
	return vol[0]*vol[1]*vol[2] + this->triangles.size();
}

void KDLeaf::getOrderedTrianlges(Vec3Df & origin, Vec3Df & dest, float distance,
		std::list< std::pair< float, std::list<unsigned int> > > & triangles){
	//triangles = std::list< std::pair< float, std::list<unsigned int> > >();
	triangles.push_back(std::pair< float, std::list<unsigned int> >(distance, this->triangles));
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

KD* KDNode::build(KDLeaf * from, unsigned int depth){
	if (depth == 0 || from->triangles.size() < 100) return from;
	int parts = 16;
	float minCost = FLT_MAX;
	KDLeaf * minL, * minR;
	for(int axis = 0; axis < 3; ++axis){
		Vec3Df step = ((from->rtr[axis] - from->lbf[axis])/parts) * Vec3Df(axis == 0, axis == 1, axis == 2);
		for(int i = 1; i < parts; ++ i){
			KDLeaf * left = new KDLeaf(from->lbf, from->rtr - (parts - i) * step);
			KDLeaf * right = new KDLeaf(from->lbf + i * step, from->rtr);
			for(std::list<unsigned int>::iterator t = from->triangles.begin();
					t != from->triangles.end(); ++t){
				left->add(*t);
				right->add(*t);
			}
			int cost = left->cost() + right->cost();
			if(cost < minCost){
				minCost = cost;
				delete minL;
				delete minR;
				minL = left;
				minR = right;
			}else{
				delete left;
				delete right;
			}
		}
	}
	return new KDNode(from->lbf, from->rtr, KDNode::build(minL, depth - 1), KDNode::build(minR, depth - 1));
}

void KDNode::getOrderedTrianlges(Vec3Df & origin, Vec3Df & dest, float distance,
		std::list< std::pair< float, std::list<unsigned int> > > & triangles){
	std::list< std::pair< float, std::list<unsigned int> > > fromLeft;
	float leftD;
	if(this->left->interSectsWithRay(origin, dest, leftD)){
		this->left->getOrderedTrianlges(origin, dest, leftD, fromLeft);
	}
	std::list< std::pair< float, std::list<unsigned int> > > fromRight;
	float rightD;
	if(this->right->interSectsWithRay(origin, dest, rightD)){
		this->right->getOrderedTrianlges(origin, dest, rightD, fromRight);
	}
	fromLeft.merge(fromRight, trianglesDistancePairComparison);
	triangles = fromLeft;
}
