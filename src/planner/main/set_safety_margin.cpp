
#include "planner_headers.h"
#include "str.h"

/*( public: )*/ void GestaltPlanner::set_safety_margin(
	const string& object_id /*( = "*" )*/,
	double margin /*( = 0.0 )*/
) {
	auto guard = state->log.log("gp.set_safety_margin",
		object_id, margin);

	if (object_id == "*") {
		for (auto& robot : state->robots) {
			robot.second.setMargin(margin);
		}
		return;
	}

	auto segments = str::split(object_id, '.');

	if (segments.size() == 1){
		auto& robot = state->getRobot(object_id);
		robot.setMargin(margin);
	}
	else if (segments.size() == 2) {
		auto& robot = state->getRobot(segments[0]);
		robot.setPartMargin(segments[1], margin);
	}
	else {
		throw std::runtime_error("invalid object_id: " + object_id);
	}
}
