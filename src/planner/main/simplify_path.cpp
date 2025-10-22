
#include "planner_headers.h"
#include "../subspace/subspace.hpp"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::simplify_path(
	const string& object_id,
	const valarray<valarray<double>>& waypoints,
	const valarray<valarray<double>>& constraints /*( = {} )*/
) {
	auto guard = state->log.log("gp.simplify_path",
		object_id, waypoints, constraints
	);

	auto& robot = state->getRobot(object_id);

	if (waypoints.size() <= 2) {
		return waypoints;
	}

	Subspace subspace(waypoints[0], constraints);

	valarray<valarray<double>> subsWaypoints = subspace.project(waypoints);

	auto checkDirect = [&subspace, &subsWaypoints, &object_id, &robot, this]
		(size_t from, size_t to) {

			auto subsSampled = LinearPointToPointTrajectory(
				subsWaypoints[from], subsWaypoints[to],
				valarray<double>(2.0 * M_PI / 180.0, subspace.subDims), // TODO
				valarray<double>(1.0e9, subspace.subDims),
				valarray<double>(1.0e9, subspace.subDims)
				) .sample(1.0, true);

			auto sampled = subspace.unproject(subsSampled);
			if (not this->check_clearance(object_id, sampled)) {
				return false;
			}

			return true;
		};

	vector<bool> keep(waypoints.size(), true);

	auto prune = [&keep, &checkDirect](const auto& prune_, size_t from, size_t to) {
		if (to - from <= 1) {
			return;
		}
		else {
			if (checkDirect(from, to)) {
				for (size_t i = from + 1; i < to; i++) {
					keep[i] = false;
				}
			}
			else {
				size_t mid = from + (to - from) / 2;
				prune_(prune_, from, mid);
				prune_(prune_, mid, to);
			}
		}
	};

	cout << "simplifying...\n" << std::flush;
	prune(prune, 0, waypoints.size() - 1);

	size_t n = 0;
	for (const auto& k: keep) { n += k; }

	cout << "keeping " << n << " out of " << waypoints.size() << " waypoints\n" << std::flush;

	valarray<valarray<double>> result(n);
	size_t j = 0;
	for (size_t i = 0; i < keep.size(); i++) {
		if (keep[i]) { result[j++] = waypoints[i]; }
	}

	state->log.log("simplified waypoints: ", result);

	return result;
}
