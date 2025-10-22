
#include "planner_headers.h"
#include "../subspace/subspace.hpp"
#include "../interpolator/interpolator.h"
#include "neldermead.hpp"



// generate velocity vectors in waypoints using heuristics similar to akima spline
// then sample to find a good scaling for each of those vectors

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::smoothen_path(
	const string& object_id,
	valarray<valarray<double>> waypoints,
	double dt,
	const valarray<double>& max_velocity,
	const valarray<double>& max_acceleration,
	const valarray<double>& max_jerk,
	const valarray<valarray<double>>& constraints /*( = {} )*/,
	bool quick /*( = true )*/
) {
	auto guard = state->log.log("gp.smoothen_path",
		object_id, waypoints, dt, max_velocity, max_acceleration, max_jerk, constraints
	);

	cout << "smoothing...\n" << std::flush;

	// https://en.wikipedia.org/wiki/Off-by-one_error#Fencepost_error
	// waypoints are fence posts, spline segments are fence sections

	// eliminate duplicates
	valarray<bool> uniqueMask(true, waypoints.size());

	for (size_t wp = 0; wp + 1 < waypoints.size(); wp++) {
		if (abs(waypoints[wp] - waypoints[wp + 1]).max() < 1e-12) {
			uniqueMask[wp + 1] = false;
		}
	}

	waypoints = waypoints[uniqueMask];

	if (waypoints.size() < 2) { return waypoints; }

	auto& robot = state->getRobot(object_id);
	size_t nwp = waypoints.size();
	size_t nseg = nwp - 1;

	Subspace subspace(waypoints[0], constraints);
	valarray<double> zeros(0.0, subspace.superDims);
	valarray<double> subZeros(0.0, subspace.subDims);

	// vectors between consecutive waypoints
	valarray<valarray<double>> betweens = diff(waypoints);

	// set distance measure for segment equal to maximum component difference
	// (can't be 0, we eliminated duplicate waypoints)
	valarray<double> distanceMeasure(betweens.size());
	for (size_t i = 0; i < betweens.size(); i++) {
		distanceMeasure[i] = abs(betweens[i]).max();
	}

	// what is the fastest the robot could travel on linear segments
	valarray<valarray<double>> maxLinearSegmentVelocities = betweens;
	for (auto& v: maxLinearSegmentVelocities) {
		v = maximizeLengthWithinCuboid(v, max_velocity);
	}

	// where two segments meet in a waypoint, how heavily does each segment's
	// velocity contribute to the velocity in the waypoint
	valarray<double> velContribution = 1.0 / distanceMeasure;


	valarray<valarray<double>> maxWaypointVelocities(nwp);
	maxWaypointVelocities[0] = zeros;
	maxWaypointVelocities[maxWaypointVelocities.size() - 1] = zeros;
	for (size_t wp = 1; wp < nwp - 1; wp++) {
		maxWaypointVelocities[wp] = (
										maxLinearSegmentVelocities[wp - 1] * velContribution[wp - 1]
										+ maxLinearSegmentVelocities[wp] * velContribution[wp]
										) / (velContribution[wp - 1] + velContribution[wp]);
	}

	valarray<double> waypointSpeedScale(0.0, nwp);

	auto trajectorize = [&](size_t seg) -> unique_ptr<ContinuousTrajectory> {

		if (waypointSpeedScale[seg] == 0 && waypointSpeedScale[seg + 1] == 0) {
			return make_unique<LinearPointToPointTrajectory>(
				waypoints[seg],
				waypoints[seg + 1],
				max_velocity,
				max_acceleration,
				max_jerk
			);
		}
		else {
			valarray<double> vStart = maxWaypointVelocities[seg] * waypointSpeedScale[seg];
			valarray<double> vEnd = maxWaypointVelocities[seg + 1] * waypointSpeedScale[seg + 1];

			return make_unique<OptimalTrajectory>(
				waypoints[seg], vStart, zeros,
				waypoints[seg + 1], vEnd, zeros,
				max_velocity,
				max_acceleration,
				max_jerk
			);
		}
	};

	auto cost = [&](size_t seg) -> double {

		auto testTrack = trajectorize(seg);
		double work = 0;

		valarray<valarray<double>> distanceSampledTestTrack = testTrack->sampleByDistance(dt, 2.0 / 180.0 * M_PI);  // TODO

		// force into constraints
		distanceSampledTestTrack = subspace.unproject(subspace.project(distanceSampledTestTrack, false, inf));

		bool bounded = robot.checkJointLimits(distanceSampledTestTrack);
		if (!bounded) {
			return inf;
		}
		bool clear = check_clearance(object_id, distanceSampledTestTrack);
		if (!clear) {
			return inf;
		}

		// cost represents integral over absolute value of acceleration
		// in order to minimize time and oscillations

		valarray<valarray<double>> timeSampledTestTrack = testTrack->sample(dt);
		timeSampledTestTrack = subspace.unproject(subspace.project(timeSampledTestTrack, false, inf));

		auto acc = diff(diff(timeSampledTestTrack));
		for (const auto& a: acc) {
			work += abs(a).sum();
		}
		return work;
	};

	double granularity = quick ? 0.333 : 0.499;
	for (size_t run = 0; run < (quick ? 1 : 3); run++) {
		for (size_t wp = 1; wp < nwp - 1; wp++) {
			double best = 0;
			double lowestCost = inf;
			for (double factor = 0; factor < 1; factor += granularity) {
				waypointSpeedScale[wp] = factor;
				double testCost1 = cost(wp - 1);
				if (std::isinf(testCost1)) { continue; }
				double testCost2 = cost(wp);
				if (std::isinf(testCost2)) { continue; }
				double testCost = testCost1 + testCost2;
				if (testCost < lowestCost) {
					best = factor;
					lowestCost = testCost;
				}
			}
			waypointSpeedScale[wp] = best;
		}
		granularity /= 2;
	}

	CompoundTrajectory compound;
	for (size_t i = 0; i < nseg; i++) {
		compound.segments.push_back(trajectorize(i));
	}

	auto result = compound.sample(dt, true);

	// force into constraints
	result = subspace.unproject(subspace.project(result, false, inf));

	// remove overshoots from constraint enforcement
	result = time_parameterize_path(
		result, dt,
		max_velocity,
		max_acceleration,
		max_jerk
	);

	// plot(result, "exports/x.py");
	// plot(diff(result, dt), "exports/v.py");
	// plot(diff(diff(result, dt), dt), "exports/a.py");
	// plot(diff(diff(diff(result, dt), dt), dt), "exports/j.py");
	return result;
}
