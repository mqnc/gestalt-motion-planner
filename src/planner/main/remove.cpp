
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::remove(
	const string& object_id
) {
	auto guard = state->log.log("gp.remove",
		object_id);

	auto& robot = state->getRobot(object_id);
	while (!robot.getChildren().empty()) {
		remove((*(robot.getChildren().begin()))->getId());
	}
	robot.setParent(nullptr, "");
	state->robots.erase(object_id);

	// prune collision ignore groups
	auto& groups = state->collisionIgnoreGroupManager.getGroups();
	for (auto group_kv = groups.begin(); group_kv != groups.end();) {
		auto& links = group_kv->second;
		for (auto link = links.begin(); link != links.end();) {
			if (state->isRobotLink(*link)) {
				link++;
			}
			else {
				link = links.erase(link);
			}
		}
		if (links.size() == 0 && group_kv->first != "__passive__") {
			group_kv = groups.erase(group_kv);
		}
		else {
			group_kv++;
		}
	}
}