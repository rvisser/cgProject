/*
 * KdTree.h
 *
 *  Created on: 18 Jun 2015
 *      Author: ruben
 */

#ifndef SOURCE_KDTREE_H_RUBENV
#define SOURCE_KDTREE_H_RUBENV

#include <utility>
#include <list>
#include "Vec3D.h"
#include "raytracing.h"

class KD {
public:
	Vec3Df lbf, rtr;
	bool interSectsWithRay(const Vec3Df & origin, const Vec3Df & dest, float & distance);
	virtual void getOrderedTrianlges(const Vec3Df & origin, const Vec3Df & dest,
			std::list< std::pair< float, std::list<unsigned int> > > & triangles) = 0;
	virtual void prettyPrint() = 0;
	//virtual ~KD() = 0;
};

bool trianglesDistancePairComparison(std::pair< float, std::list<unsigned int> > first,
		std::pair< float, std::list<unsigned int> > second);

class KDLeaf: public KD {
public:
	std::list<unsigned int> triangles;
	KDLeaf(Vec3Df lbf, Vec3Df rtr);
	KDLeaf();
	void add(unsigned int triangle);
	float cost();
	void optimizeBox();
	void prettyPrint();
	void getOrderedTrianlges(const Vec3Df & origin, const Vec3Df & dest,
				std::list< std::pair< float, std::list<unsigned int> > > & triangles);
};

class KDNode: public KD {
private:
	KD * left, * right;
public:
	KDNode(Vec3Df lbf, Vec3Df rtr, KD* left, KD* right);
	static KD* build(KDLeaf * from, unsigned int depth);
	void prettyPrint();
	void getOrderedTrianlges(const Vec3Df & origin, const Vec3Df & dest,
				std::list< std::pair< float, std::list<unsigned int> > > & triangles);
};



#endif /* SOURCE_KDTREE_H_RUBENV */
