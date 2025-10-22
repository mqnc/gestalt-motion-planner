
#include "planner_headers.h"
#include "render/sceneexport.hpp"

/*( public: )*/ vector<Collision> GestaltPlanner::find_collisions(
	const string& object_id /*( = "__world__" )*/,
	const valarray<valarray<double>>& trajectory /*( = {} )*/
) {
	auto guard = state->log.log("gp.find_collisions",
		trajectory);

	auto& robot = state->getRobot(object_id);

	auto restoreJoints = robot.getJointPositions();
	OnScopeExit jointRestorer(
		[&]() {set_joint_positions(object_id, restoreJoints);});

	robot.setActuationState(true);
	OnScopeExit deactuator(
		[&]() {robot.setActuationState(false);});

	update_collision_bitmasks();

	vector<Collision> result;

	if (trajectory.size() == 0) {
		auto report = getCollisionInfo(*state);
		for (const auto& col : report) {
			result.push_back({ -1, col.link1, col.link2,
				{col.position.x(), col.position.y(), col.position.z()} });
		}
	}
	else {
		for (auto&& [step, positions] : enumerate(trajectory)) {
			robot.setJointPositions(positions);

			auto report = getCollisionInfo(*state);
			for (const auto& col : report) {
				result.push_back({ (int)step, col.link1, col.link2,
					{col.position.x(), col.position.y(), col.position.z()} });
			}
		}
	}
	return result;
}
