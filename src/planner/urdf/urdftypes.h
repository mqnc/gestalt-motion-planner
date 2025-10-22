
#pragma once

#include "common.h"

struct UrdfTrafo {
	valarray<double> xyz{ 0,0,0 };
	valarray<double> rpy{ 0,0,0 };
};

struct UrdfCollisionGeometry {
	string type;
	UrdfTrafo origin;
	valarray<double> size;
	valarray<double> meshScale;
	double radius;
	double length;
	string filename;
};

struct UrdfLink {
	const string name;
	vector<UrdfCollisionGeometry> collisionGeometries;
};

struct UrdfJoint {
	const string name;
	string type;
	string parent;
	string child;
	UrdfTrafo origin;
	valarray<double> axis;
	array<double, 2> limit;
};

struct ByName {
	const std::string& operator() (const UrdfLink& l) {
		return l.name;
	}
	const std::string& operator() (const UrdfJoint& j) {
		return j.name;
	}
};

struct UrdfRobot {
	const string name;
	vector<string> defaultJointSelection;
	Registry<UrdfLink, ByName> links;
	Registry<UrdfJoint, ByName> joints;
};