
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::set_joint_positions(
	const string& object_id,
	const valarray<double>& joint_positions /*( = {} )*/
) {
	auto guard = state->log.log("gp.set_joint_positions",
		object_id, joint_positions);

	auto& robot = state->getRobot(object_id);
	robot.setJointPositions(joint_positions);
}
