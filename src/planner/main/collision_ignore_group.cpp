
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::create_collision_ignore_group(
	const string& name,
	const vector<string>& members
) {
	auto guard = state->log.log("gp.create_collision_ignore_group",
		name, members);

	state->assertIdFree(name);

	unordered_set<string> links;
	for (const auto& item : members) {

		if (state->robots.count(item) > 0) {
			// add all links of a robot
			for (const auto& part : state->robots.at(item).getParts()) {
				links.insert(item + "." + part->linkId);
			}
		}
		else if (state->collisionIgnoreGroupManager.getGroups().count(item) > 0) {
			// add all links from another collision ignore group
			for (const auto& link :
				state->collisionIgnoreGroupManager.getGroups().at(item)) {

				links.insert(link);
			}
		}
		else if (state->isRobotLink(item)) {
			// add specific link of specific robot
			links.insert(item);
		}
		else {
			throw runtime_error(string() +
				"cannot find object or link or collision ignore group named \""
				+ item + "\"");
		}
	}

	state->collisionIgnoreGroupManager.createGroup(name, links);
};

/*( public: )*/ void GestaltPlanner::delete_collision_ignore_group(
	const string& name
) {
	auto guard = state->log.log("gp.delete_collision_ignore_group",
		name);

	state->collisionIgnoreGroupManager.deleteGroup(name);
};

/*( public: )*/ std::unordered_map<string, std::unordered_set<string>>
GestaltPlanner::get_collision_ignore_groups(
	const string& active_object /*( = "" )*/
) {
	auto guard = state->log.log("gp.get_collision_ignore_groups",
		active_object);

	if (active_object != "") {
		state->getRobot(active_object).setActuationState(true);
	}
	OnScopeExit deactuator(
		[&]() {
			for (auto& [robotId, robot] : state->robots) {
				robot.setActuationState(false);
			}
		});

	update_collision_bitmasks();

	return state->collisionIgnoreGroupManager.getGroups();

}

/*( private: )*/ void GestaltPlanner::update_collision_bitmasks() {
	auto guard = state->log.log("gp.update_collision_bitmasks");

	vector<string> passive;

	// collect all passive robots
	for (const auto& [robotId, robot] : state->robots) {
		for (const auto& part : robot.getParts()) {
			if (not robot.isActuated) {
				passive.push_back(robotId + "." + part->linkId);
			}
		}
	}

	// update passive group
	state->collisionIgnoreGroupManager.resetGroup("__passive__", passive);
	state->collisionIgnoreGroupManager.generateBitMasks();

	// assign bitmasks to robots
	for (auto& [robotId, robot] : state->robots) {
		for (auto& part : robot.getMutableParts()) {
			part->collisionBitMask = state->collisionIgnoreGroupManager.getBitMask(
				robotId + "." + part->linkId);
		}
	}

}