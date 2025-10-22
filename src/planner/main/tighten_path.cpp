
#include "planner_headers.h"
#include "../subspace/subspace.hpp"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::tighten_path(
	const string& object_id,
	const valarray<valarray<double>>& waypoints,
	const valarray<valarray<double>>& constraints /*( = {} )*/,
	size_t iterations /*( = 3 )*/
) {
	auto guard = state->log.log("gp.tighten_path",
		object_id, waypoints, constraints
	);

	auto& robot = state->getRobot(object_id);

	if (waypoints.size() <= 2) {
		return waypoints;
	}

	Subspace subspace(waypoints[0], constraints);

	valarray<valarray<double>> subsWaypoints = subspace.project(waypoints);

	auto check = [&subspace, &object_id, &robot, this]
	(valarray<valarray<double>>& subsWaypoints, size_t from, size_t to) {

		// we might leave the convex subspace when touching individual parameters
		for(size_t i = from; i <= to; i++){
			if(not robot.checkJointLimits(subspace.unproject(subsWaypoints[i]))){
				return false;
			}
		}

		for(size_t i = from; i < to; i++){
			auto subsSampled = LinearPointToPointTrajectory(
				subsWaypoints[i], subsWaypoints[i + 1],
				valarray<double>(2.0 * M_PI / 180.0, subspace.subDims), // TODO
				valarray<double>(1.0e9, subspace.subDims),
				valarray<double>(1.0e9, subspace.subDims)
			).sample(1.0, true);

			auto sampled = subspace.unproject(subsSampled);
			if(not this->check_clearance(object_id, sampled)){
				return false;
			}
		}
		
		return true;
	};

	auto smoothen = [&](double neighborWeight) {
		// std::cout << "neighborWeight: " << neighborWeight << std::endl;
		for (const size_t parity : {0, 1}) {
			for (size_t i = 1 + parity; i < subsWaypoints.size() - 1; i += 2) {
				auto backup = subsWaypoints[i];

				subsWaypoints[i] =
					(1.0 - neighborWeight) * subsWaypoints[i]
					+ 0.5 * neighborWeight * subsWaypoints[i - 1]
					+ 0.5 * neighborWeight * subsWaypoints[i + 1];

				if (!check(subsWaypoints, i - 1, i + 1)) { // didn't work, undo and try individual parameters
					// std::cout << "X   ";
					subsWaypoints[i] = backup;

					for (size_t k = 0; k < subspace.subDims; k++) {
						auto backup = subsWaypoints[i][k];

						subsWaypoints[i][k] =
							(1.0 - neighborWeight) * subsWaypoints[i][k]
							+ 0.5 * neighborWeight * subsWaypoints[i - 1][k]
							+ 0.5 * neighborWeight * subsWaypoints[i + 1][k];

						if (!check(subsWaypoints, i - 1, i + 1)) {
							subsWaypoints[i][k] = backup;
							// std::cout << "x";
						}
						else{
							// std::cout << "v";
						}
					}
				}
				else{
					// std::cout << "V   ";
				}
			}
		}
		// std::cout << std::endl;
	};

	// try clamping each joint to the corridor between its start and target value
	auto clamp = [&]() {
		for (size_t k = 0; k < subspace.subDims; k++) {

			auto backup = subsWaypoints;
			const size_t nWP = subsWaypoints.size();

			double lower = std::min(subsWaypoints[0][k], subsWaypoints[nWP - 1][k]);
			double upper = std::max(subsWaypoints[0][k], subsWaypoints[nWP - 1][k]);

			for (size_t i = 1; i < nWP - 1; i++) {
				subsWaypoints[i][k] = std::clamp(subsWaypoints[i][k], lower, upper);
			}

			if (!check(subsWaypoints, 0, nWP - 1)) {
				subsWaypoints = backup;
				continue;
			}
		}
	};

	state->log.log("gp.tighten_path", "clamp");
	cout << "clamping...\n" << std::flush;
	clamp();
	cout << "tightening...\n" << std::flush;
	for (size_t iteration = 0; iteration < iterations; iteration++) {
		double neighborWeight = 1.0 / pow(1.2, iteration);
		state->log.log("gp.tighten_path", "smoothen");
		smoothen(neighborWeight);
	}
	//state->log.log("gp.tighten_path", "clamp");
	//clamp();

	auto result = subspace.unproject(subsWaypoints);

	state->log.log("tightened waypoints: ", result);

	return result;
}
