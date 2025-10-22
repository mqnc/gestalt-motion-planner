
#include "planner_headers.h"

/*( public: )*/ bool GestaltPlanner::check_kinematic_feasibility(
	const valarray<valarray<double>>& trajectory,
	double dt,
	const valarray<double>& max_velocity,
	const valarray<double>& max_acceleration,
	const valarray<double>& max_jerk
) {
	auto guard = state->log.log("gp.check_kinematic_feasibility",
		trajectory, max_velocity, max_acceleration, max_jerk);

	auto pos = trajectory;
	auto vel = diff(pos) / dt;
	auto acc = diff(vel) / dt;
	auto jrk = diff(acc) / dt;

	// velocity limit
	for (const auto& vs : vel) {
		for (auto&& [v, vmax] : zip(vs, max_velocity)) {
			if (fabs(v) >= vmax) {
				return false;
			}
		}
	}

	// acceleration limit
	for (const auto& as : acc) {
		for (auto&& [a, amax] : zip(as, max_acceleration)) {
			if (fabs(a) >= amax) {
				return false;
			}
		}
	}

	// jerk limit
	for (const auto& js : jrk) {
		for (auto&& [j, jmax] : zip(js, max_jerk)) {
			if (fabs(j) >= jmax) {
				return false;
			}
		}
	}

	return true;
}
