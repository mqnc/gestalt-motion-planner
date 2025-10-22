
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::set_base_pose(
	const string& object_id,
	const Pose& pose /*( = PoseUpdate{} )*/,
	const string& parent_object_id /*( = "" )*/,
	const string parent_link /*( = "__root__" )*/
) {
	auto guard = state->log.log("gp.set_base_pose",
		object_id, pose, parent_object_id, parent_link);

	auto& robot = state->getRobot(object_id);

	if (parent_object_id == "") { // parent does not change
		robot.setBaseTrafo(btTransformFromPose(pose));
	}
	else {
		auto& parent = state->getRobot(parent_object_id);

		CollisionRobot* ancestor = &parent;
		while (ancestor != nullptr) {
			if (ancestor == &robot) {
				throw runtime_error("attempted to set_base_pose with a circular parent reference");
			}
			ancestor = ancestor->getParent().first;
		}

		btTransform relPose =
			parent.getPartTrafoInWorld(parent_link).inverseTimes(
				robot.getPartTrafoInWorld("__root__"));

		updateBulletTrafo(relPose, btTransformFromPose(pose));

		robot.setParent(&parent, parent_link);
		robot.setBaseTrafo(relPose);
	}
}
