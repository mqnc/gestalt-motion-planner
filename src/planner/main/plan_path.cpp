
#include "planner_headers.h"
#include "pathfinder/planners.hpp"
#include "zip.h"
#include "../subspace/subspace.hpp"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::plan_path(
	const string& object_id,
	const valarray<valarray<double>>& target_joint_positions /*( = {} )*/,
	valarray<double> start_joint_positions /*( = {} )*/,
	valarray<double> max_sampling_step_size /*( = {} )*/,
	const valarray<valarray<double>>& waypoint_suggestions /*( = {} )*/,
	double jiggle /*( = 1e-6 )*/,
	const valarray<valarray<double>>& constraints /*( = {} )*/,
	double constraint_tolerance /*( = 1e-6 )*/,
	const string& planner /*( = "RRTConnect" )*/,
	const std::map<string, string>& planner_params /*( = {} )*/,
	size_t maxChecks /*( = 5000 )*/,
	double timeout /*( = -1.0 )*/,
	size_t random_seed /*( = 0 )*/,
	bool simplify /*( = true )*/,
	bool tighten /*( = true )*/
) {

	auto guard = state->log.log("gp.plan_path",
		object_id,
		target_joint_positions, start_joint_positions,
		max_sampling_step_size, waypoint_suggestions,
		jiggle, constraints, constraint_tolerance,
		planner, planner_params,
		maxChecks, timeout, random_seed,
		simplify, tighten);

	valarray<valarray<double>> result;

	{ // inner scope to destruct the collision checker before simplifying and tightening

		bool verbose = false;

		auto& robot = state->getRobot(object_id);

		if (start_joint_positions.size() == 0) {
			start_joint_positions = robot.getJointPositions();
		}

		size_t numJoints = robot.getJointSelection().size();
		bool constrained = constraints.size() > 0;
		if (target_joint_positions.size() < 1) {
			throw runtime_error("plan_path: no targets specified");
		}

		if (start_joint_positions.size() != numJoints) {
			throw runtime_error("plan_path: dimension mismatch");
		}
		for (const auto& t: target_joint_positions) {
			if (t.size() != numJoints) {
				throw runtime_error("plan_path: dimension mismatch");
			}
		}

		Subspace subspace(start_joint_positions, constraints);
		auto startValues = subspace.project(start_joint_positions, false, constraint_tolerance);
		auto targetValues = subspace.project(target_joint_positions, false, constraint_tolerance);
		auto waypointValues = subspace.project(waypoint_suggestions, false, constraint_tolerance);

		valarray<double> minJoints = valarray<double>(numJoints);
		valarray<double> maxJoints = valarray<double>(numJoints);
		for (size_t j = 0; j < numJoints; j++) {
			minJoints[j] = robot.getPartByJointIndex(j).limits[0];
			maxJoints[j] = robot.getPartByJointIndex(j).limits[1];
		}
		valarray<double> maxStepSize = robot.getMaxJointStepSizes(max_sampling_step_size);

		valarray<double> minValues;
		valarray<double> maxValues;

		if (constrained) {
			// big todo: -2pi..2pi is kind of arbitrary
			// the actual limits form a simplex in which it is difficult to sample
			// eigen computes a kernel in which max(abs(v))==1 for all basis vectors
			// so -2pi..2pi corresponds to a full revolution of the fastest joint
			// in each direction
			minValues = valarray<double>(-2.0 * M_PI, subspace.subDims);
			maxValues = valarray<double>(2.0 * M_PI, subspace.subDims);
		}
		else {
			minValues = minJoints;
			maxValues = maxJoints;
		}

		auto restoreJoints = robot.getJointPositions();
		OnScopeExit jointRestorer(
			[&]() { set_joint_positions(object_id, restoreJoints); });

		robot.setActuationState(true);
		OnScopeExit deactuator(
			[&]() { robot.setActuationState(false); });

		update_collision_bitmasks();

		CollisionChecker checker(state->extractBulletObjectsAndBitMasks());

		function<bool(const valarray<double>&)> testCallback =
			[&](const valarray<double>& values) -> bool {

			auto joints = subspace.unproject(values);
			if (!robot.checkJointLimits(joints)) { return false; }
			robot.setJointPositions(joints);
			bool result = checker.checkCollisions().numCollisions == 0;
			return result;
		};

		function<double(const valarray<double>&, const valarray<double>&)> distanceCallback =
			[&](const valarray<double>& values1, const valarray<double>& values2) -> double {

			valarray<double> delta = subspace.unproject(values2) - subspace.unproject(values1);
			return norm(delta);
		};

		function<double(const valarray<double>&, const valarray<double>&)> subdivideCallback =
			[&](const valarray<double>& values1, const valarray<double>& values2) -> double {

			valarray<double> delta = subspace.unproject(values2) - subspace.unproject(values1);

			double radicand = 0;
			for (size_t i = 0; i < numJoints; i++) {
				double numSegmentsForThisJoint = delta[i] / maxStepSize[i];
				radicand += numSegmentsForThisJoint * numSegmentsForThisJoint;
			}
			return ceil(sqrt(radicand));
		};

		auto path = findPath(
			startValues,
			targetValues,
			minValues, maxValues,
			testCallback,
			distanceCallback,
			subdivideCallback,
			planner,
			planner_params,
			waypointValues,
			jiggle,
			maxChecks,
			timeout,
			random_seed,
			verbose
		);

		result = subspace.unproject(path);
	}

	state->log.log("found waypoints: ", result);

	if (simplify) {
		result = simplify_path(object_id, result, constraints);
	}

	if (tighten) {
		result = tighten_path(object_id, result, constraints);
	}

	return result;
}


/*( public: )*/ std::map<string, PlannerInfo> GestaltPlanner::get_planner_info() {
	auto guard = state->log.log("gp.get_planner_info");
	return getPlannerInfo();
}
