
#include "planner_headers.h"

/*( private: )*/ void GestaltPlanner::spawn_world() {
	auto guard = state->log.log("gp.spawn_world");

	string object_id = "__world__";

	string urdf = R"(
		<?xml version="1.0"?>
		<robot name="world">
			<link name="origin">
			</link>
		</robot> 
	)";

	state->robotTemplates.emplace(object_id,
		make_shared<CollisionRobotTemplate>(object_id, parseUrdf(urdf)));

	state->robots.emplace(object_id,
		CollisionRobot(*state->robotTemplates.at(object_id), object_id));

}
