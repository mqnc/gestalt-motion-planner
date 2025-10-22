
#pragma once

#include "common.h"

#include "btBulletCollisionCommon.h"

#include "collision/shapes.h"
#include "collision/collisionrobot.h"
#include "collision/collisionchecker.h"

string vectorToHtml(double x, double y, double z);

string vectorToHtml(const btVector3& vec);

string trafoToHtml(const btTransform& trafo);

class HtmlConverter :public ShapeVisitor {
	btTransform trafo;
	string material;
	stringstream result;
public:

	HtmlConverter(const btTransform& trafo, string material) :
		trafo{ trafo },
		material{ material }{}

	string getResult() {
		return result.str();
	}

	virtual void visit(CompoundShape* shape);
	virtual void visit(BoxShape* shape);
	virtual void visit(CylinderZShape* shape);
	virtual void visit(SphereShape* shape);
	virtual void visit(CapsuleZShape* shape);
	virtual void visit(ConvexHullShape* shape);
	virtual void visit(ConcaveTriangleMeshShape* shape);
	virtual void visit(MultiSphereShape* shape);
	virtual void visit(VisualMeshShape* shape);
};

string sceneToHtml(
	const dict<CollisionRobot>& robots,
	const CollisionChecker::CollisionReport& collisions
);
