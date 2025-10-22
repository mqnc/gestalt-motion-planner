
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::reset() {
	auto guard = state->log.log("gp.reset");

	while (!state->robots.empty()) {
		// remove removes an object and all its children, so we need a while loop
		remove(state->robots.begin()->first);
	}
	state->robotTemplates.clear();
	state->clearCache();
	state->collisionIgnoreGroupManager.clear();

	// group with all passive objects so they are not tested for collisions
	// placeholder so the user does not exceed the maximum amount of collision ignore groups
	state->collisionIgnoreGroupManager.createGroup<vector>("__passive__", {});

	spawn_world();

}
