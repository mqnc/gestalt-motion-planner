
#include "planner_headers.h"

// much faster than find_collisions()
// only reports true if there were no collisions
// randomizes waypoint checking order
// stops on first collision encounter
/*( public: )*/ bool GestaltPlanner::check_clearance(
	const string& object_id /*( = "__world__" )*/,
	valarray<valarray<double>> trajectory /*( = {} )*/
) {
	auto guard = state->log.log("gp.check_clearance",
		object_id, trajectory);

	auto& robot = state->getRobot(object_id);

	robot.setActuationState(true);
	OnScopeExit deactuator(
		[&]() {robot.setActuationState(false);});
	
	if (trajectory.size() == 0) {
		// just check the current state
		update_collision_bitmasks();
		return CollisionChecker(state->extractBulletObjectsAndBitMasks())
			.checkCollisions().numCollisions == 0;
	}
	else {
		std::shuffle(begin(trajectory), end(trajectory),
			std::default_random_engine{});

		auto restoreJoints = robot.getJointPositions();
		OnScopeExit jointRestorer(
			[&]() {set_joint_positions(object_id, restoreJoints);});

		update_collision_bitmasks();

		for (const auto& positions : trajectory) {
			robot.setJointPositions(positions);

			if (
				CollisionChecker(state->extractBulletObjectsAndBitMasks())
				.checkCollisions().numCollisions > 0
				) {

				return false;
			}
		};

		return true;
	}
}
