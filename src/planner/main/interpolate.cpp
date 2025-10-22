
#include "planner_headers.h"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::interpolate(
	const valarray<valarray<double>>& waypoints,
	double dt,
	const valarray<double>& max_velocity,
	const valarray<double>& max_acceleration /*( = {} )*/,
	const valarray<double>& max_jerk /*( = {} )*/,
	double safety_factor /*( = 1.0 )*/
) {
	auto guard = state->log.log("gp.interpolate",
		waypoints, dt, max_velocity, max_acceleration, max_jerk, safety_factor);

	const size_t dof = max_velocity.size();
	auto maxAcc = max_acceleration.size() > 0 ? max_acceleration : valarray<double>(1.0e9, dof);
	auto maxJrk = max_acceleration.size() > 0 ? max_jerk : valarray<double>(1.0e9, dof);

	for (const auto& wp: waypoints) {
		if (wp.size() != dof) {
			throw runtime_error("interpolate: dimension mismatch");
		}
	}

	if (maxAcc.size() != dof || maxJrk.size() != dof ) {
		throw runtime_error("interpolate: dimension mismatch");
	}

	if (safety_factor < 1) {
		cout << "warning: safety_factor should be >= 1" << "\n";
	}

	if (waypoints.size() <= 1) {
		return waypoints;
	}

	CompoundTrajectory traj;
	traj.segments.reserve(waypoints.size() - 1);
	for (size_t i = 1; i < waypoints.size(); i++) {
		traj.segments.push_back(
			make_unique<LinearPointToPointTrajectory>(
				waypoints[i - 1],
				waypoints[i],
				(1.0 / safety_factor) * max_velocity,
				(1.0 / safety_factor) * maxAcc,
				(1.0 / safety_factor) * maxJrk
				)
		);
	}
	return traj.sample(dt, true);
}
