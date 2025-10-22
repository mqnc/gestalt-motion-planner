
#include "planner_headers.h"

// leave joints empty to use all joints from robot.urdf or selected joints from config.yaml
/*( public: )*/ void GestaltPlanner::select_joints(
	const string& object_id,
	const vector<string>& joints /*( = {} )*/ 
) {
	auto guard = state->log.log("gp.select_joints",
		object_id, joints);

	state->getRobot(object_id).selectJoints(joints);
}

/*( public: )*/ vector<string> GestaltPlanner::get_joint_selection(
	const string& object_id
) {
	auto guard = state->log.log("gp.get_joint_selection",
		object_id);

	return state->getRobot(object_id).getJointSelection();
}
