
#pragma once

#include "planner_headers.h"

class PlannerState {
public:
	Log log;

	dict<shared_ptr<CollisionRobotTemplate>> robotTemplates;

	dict<CollisionRobot> robots;

	CollisionIgnoreGroupManager collisionIgnoreGroupManager;

	void assertIdFree(string id);

	bool isRobotLink(const string& id);

	CollisionRobot& getRobot(string id);

	void cacheFile(const string& name, const string& content, bool isBase64=false);

	string loadFile(const string& name, bool binary = false);

	bool isCached(const string& name) const;

	const dict<string>& getCache() const;

	void clearCache();

	vector<pair<btCollisionObject*, BitMask>> extractBulletObjectsAndBitMasks();

private:
	dict<string> fileCache;
};
