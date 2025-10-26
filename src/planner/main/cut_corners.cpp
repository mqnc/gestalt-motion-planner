
#include "planner_headers.h"
#include "../subspace/subspace.hpp"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::cut_corners(
	const string& object_id,
	const valarray<valarray<double>>& waypoints,
	vector<double> cutoff_ratios /*( = {0.5, 0.25} )*/,
	double angle_threshold /*( = 60.0 * 3.1416/180.0 )*/,
	double distance_threshold /*( = 0.2 )*/
) {
	auto guard = state->log.log("gp.cut_corners",
		object_id, waypoints, cutoff_ratios, angle_threshold, distance_threshold
	);

	auto& robot = state->getRobot(object_id);

	if (waypoints.size() <= 2) {
		return waypoints;
	}

	for (double r:cutoff_ratios){
		if (r <= 0.0 || r > 0.5){
			throw runtime_error("cut_corners: cutoff_ratios must be >0 and <=0.5");
		}
	}
	std::sort(cutoff_ratios.begin(), cutoff_ratios.end(), std::greater<double>());

	if (angle_threshold <= 0.0 || angle_threshold >= M_PI){
		throw runtime_error("cut_corners: angle_threshold must be >0 and <pi");
	}
	double cosAngleThreshold = cos(angle_threshold);

	size_t nDims = waypoints[0].size();

	vector<valarray<double>> newWaypoints;
	newWaypoints.push_back(waypoints[0]);

	for (size_t i = 1; i+1 < waypoints.size(); i++){

		valarray<double> dirIn = waypoints[i] - waypoints[i-1];
		dirIn /= norm(dirIn);

		valarray<double> dirOut = waypoints[i+1] - waypoints[i];
		dirOut /= norm(dirOut);

		double cosAngle = (dirIn * dirOut).sum();
		if (cosAngle > cosAngleThreshold){
			// direction change is smaller than angle threshold
			// cout << "angle too obtuse\n" << std::flush;
			continue;
		}

		bool cut = false;
		for (double r:cutoff_ratios){
			// cout << "testing ratio " << r << "\n" << std::flush;

			valarray<double> from = waypoints[i-1] * r + waypoints[i] * (1.0-r);
			valarray<double> to = waypoints[i] * (1.0-r) + waypoints[i+1] * r;

			double originalDistance = distance(from, waypoints[i]) + distance(waypoints[i], to);
			double newDistance = distance(from, to);
			double savedDistance = originalDistance - newDistance;

			if (savedDistance < distance_threshold){
				// cout << "too little saving " << savedDistance << "\n" << std::flush;
				break;
			}

			auto sampled = LinearPointToPointTrajectory(
				from, to,
				valarray<double>(2.0 * M_PI / 180.0, nDims), // TODO
				valarray<double>(1.0e9, nDims),
				valarray<double>(1.0e9, nDims)
			).sample(1.0, true);


			if (check_clearance(object_id, sampled)){
				// if cutoff_ratio is 0.5, the last to-point can coincide with this from-point
				if (distance(newWaypoints.back(), from) > 1e-9){
					// cout << "inserting 'from' waypoint\n" << std::flush;
					newWaypoints.push_back(from);
				}
				// cout << "inserting 'to' waypoint\n" << std::flush;
				newWaypoints.push_back(to);
				cut = true;
				break;
			}
			else{
				// cout << "collision\n" << std::flush;
			}
		}
		if (!cut){
			newWaypoints.push_back(waypoints[i]);
		}
	}

	newWaypoints.push_back(waypoints[waypoints.size()-1]);

	return valarray<valarray<double>>(newWaypoints.data(), newWaypoints.size());
}
