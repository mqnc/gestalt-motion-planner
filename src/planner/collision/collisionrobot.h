
#pragma once

#include "common.h"

#include "urdf/urdf.h"
#include "stl_reader_mod.h"
#include "btBulletCollisionCommon.h"
#include "shapes.h"
#include "multibody.h"
#include "collisionignoregroup.hpp"
//#include "encapsulator/encapsulate.h"

auto btTransformFromUrdf(const UrdfTrafo& ut);

class CollisionRobotTemplate
{
	const string id;
	MultiBodyDescriptor partDescriptors;
	vector<string> selectedJoints;
	dict<vector<string>> collisionIgnoreGroups;

public:
	CollisionRobotTemplate(
		const string& id,
		const UrdfRobot& urdfRobot,
		const dict<string>& meshSources = {},
		const vector<string>& selectedJoints = {},
		const dict<vector<string>>& collisionIgnoreGroups = {},
		const bool encapsulate = false);

	CollisionRobotTemplate() {}

	friend class CollisionRobot;
	friend class CollisionRobotSerializer;
};

class CollisionRobot
{
	const string id;

	CollisionRobot* parent = nullptr;
	Part* parentPart = nullptr;
	std::unordered_set<CollisionRobot*> children;

	MultiBody parts;
	vector<string> preSelectedJoints;
	vector<string> selectedJoints;
	vector<Part*> jointMap;

	// id gets prefixed to all groups and members during init
	dict<vector<string>> collisionIgnoreGroups;

	void init();

public:
	Part* root = nullptr;
	bool isActuated = false;

	CollisionRobot(
		const CollisionRobotTemplate& tmpl,
		const string& id);

	CollisionRobot() {};

	string getId() { return id; }

	const pair<CollisionRobot*, Part*> getParent() const;

	void setParent(CollisionRobot* newParent, string newParentLinkName);

	const btTransform& getParentTrafo();

	void setBaseTrafo(const btTransform& trafo);

	btTransform getPartTrafoInWorld(string link = "__root__");

	void selectJoints(const vector<string>& ids);

	const vector<string>& getJointSelection() const;

	valarray<double> getJointPositions() const;

	void setJointPositions(const valarray<double>& jointValues);

	bool checkJointLimits(const valarray<double>& jointValues) const;
	bool checkJointLimits(const valarray<valarray<double>>& jointTrajectory) const;
	void getJointLimits(valarray<double>& upperLimits, valarray<double>& lowerLimits) const;
	valarray<double> getMaxJointStepSizes(valarray<double> override) const;

	void setActuationState(bool actuated);

	std::unordered_set<CollisionRobot*>& getChildren();

	const Part& getPartByJointId(string id) const;
	Part& getPartByJointId(string id);
	const Part& getPartByJointIndex(size_t index) const;
	Part& getPartByJointIndex(size_t index);
	const Part& getPartByLinkId(string id) const;
	Part& getPartByLinkId(string id);

	const MultiBody& getParts() const;
	MultiBody& getMutableParts();

	void setMargin(double margin);
	void setPartMargin(string id, double margin);

	const dict<vector<string>>& getCollisionIgnoreGroups() const;

	friend class CollisionRobotTest;
	friend class CollisionRobotSerializer;
};
