
#include "planner_headers.h"
#include "base64.hpp"

void PlannerState::assertIdFree(string id) {
	if (robots.count(id) > 0) {
		throw runtime_error("an object named \"" + id + "\" already exists");
	}
	if (collisionIgnoreGroupManager.getGroups().count(id) > 0) {
		throw runtime_error("a collision ignore group named \""
			+ id + "\" already exists");
	}
};

bool PlannerState::isRobotLink(const string& id) {
	vector<string> parts = str::split(id, '.');
	if (
		parts.size() == 2
		&& robots.count(parts[0]) > 0
		&& robots.at(parts[0]).getParts().contains(byLink, parts[1])) {

		return true;
	}
	else {
		return false;
	}
}


CollisionRobot& PlannerState::getRobot(string id) {
	if (robots.count(id) == 0) {
		cout << "attempted to access object with id \""
			 << id << "\" which does not exist; known objects: ";
		for (const auto& robot: robots) {
			cout << '"' << robot.first << "\" ";
		}
		cout << "\n";
		// still trigger standard exception below
	}
	return robots.at(id);
}

void PlannerState::cacheFile(const string& name, const string& content, bool isBase64) {
	if (isBase64) {
		fileCache[name] = base64::from_base64(content);
	}
	else {
		fileCache[name] = content;
	}
}

string PlannerState::loadFile(const string& name, bool binary) {
	if (!fileCache.count(name)) {
		//cout << "reading " << name << " from disk" << "\n";
		fileCache[name] = str::load(name, binary);
		if (log.isLogging()) {
			log.log("gp.cache_file", name, base64::to_base64(fileCache[name]), true);
		}
	}
	else {
		//cout << "reading " << name << " from cache" << "\n";
	}
	return fileCache[name];
}

bool PlannerState::isCached(const string& name) const {
	return fileCache.count(name) > 0;
}

const dict<string>& PlannerState::getCache() const{
	return fileCache;
}

void PlannerState::clearCache() {
	fileCache.clear();
}

vector<pair<btCollisionObject*, BitMask>> PlannerState::extractBulletObjectsAndBitMasks() {
	vector<pair<btCollisionObject*, BitMask>> result;

	for (auto& [robotId, robot]: robots) {
		for (auto& part: robot.getMutableParts()) {
			result.push_back({
				&(part->bulletObject),
				part->collisionBitMask
			});
		}
	}

	return result;
}
