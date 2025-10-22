
#include "findpath.h"
#include "planners.hpp"

#include "common.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <boost/math/special_functions/prime.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class ModdedRealVectorStateSpace: public ob::RealVectorStateSpace {
	function<double(
		const valarray<double>&,
		const valarray<double>&
		)> distanceCallback;
	function<unsigned int(
		const valarray<double>&,
		const valarray<double>&
		)> subdivisionCallback;
public:
	ModdedRealVectorStateSpace(
		unsigned int dim,
		function<double(
			const valarray<double>&,
			const valarray<double>&
			)> distfn,
		function<unsigned int(
			const valarray<double>&,
			const valarray<double>&
			)> subdivfn
		):
		RealVectorStateSpace(dim),
		distanceCallback {distfn},
		subdivisionCallback {subdivfn} {}

	double distance(const ob::State* state1, const ob::State* state2) const override {
		const valarray<double> va1(state1->as<StateType>()->values, getDimension());
		const valarray<double> va2(state2->as<StateType>()->values, getDimension());
		return distanceCallback(va1, va2);
	}

	unsigned int validSegmentCount(const ob::State* state1, const ob::State* state2) const override {
		const valarray<double> va1(state1->as<StateType>()->values, getDimension());
		const valarray<double> va2(state2->as<StateType>()->values, getDimension());
		return subdivisionCallback(va1, va2);
	};
};


// this sampler returns predefined suggestions first and also every some samples
template <typename T>
class BiasedSampler: public T {
	const ob::StateSpace* space_;
	size_t callCounter = 0;
	valarray<valarray<double>> suggestions;
	size_t suggestionInterval = 15;

public:
	template <typename... Args>
	BiasedSampler(
		const ob::StateSpace* space,
		valarray<valarray<double>> suggestions,
		size_t suggestionInterval,
		Args&&... args
		):
		T(space, std::forward<Args>(args)...),
		space_ {space},
		suggestions {suggestions},
		suggestionInterval {suggestionInterval} {}

	void sampleUniform(ob::State* state) {
		int suggest = -1;
		if (callCounter < suggestions.size()) {
			suggest = callCounter;
		}
		else if (suggestions.size() > 0 && callCounter % suggestionInterval == 0) {
			suggest = (callCounter / suggestionInterval - 1) % suggestions.size();
		}

		if (suggest != -1) {
			const unsigned int dim = space_->getDimension();
			auto* rstate = static_cast<typename ob::RealVectorStateSpace::StateType*>(state);
			for (unsigned int i = 0; i < dim; ++i) {
				rstate->values[i] = suggestions[suggest][i];
			}
		}
		else {
			T::sampleUniform(state);
		}
		callCounter++;
	}
};


class AdditiveRecurrenceSequence: public ob::DeterministicSequence {
	size_t k = 0;
	valarray<double> s0;
	valarray<double> alphae;
public:

	AdditiveRecurrenceSequence(unsigned int dims): DeterministicSequence(dims) {
		s0.resize(dims);
		alphae.resize(dims);
		for (unsigned int i = 0; i < dims; i++) {
			s0[i] = 0.5;
			alphae[i] = sqrt(boost::math::prime(i));
		}
	}
	std::vector<double> sample() override {
		std::vector<double> result(dimensions_);
		for (unsigned int i = 0; i < dimensions_; i++) {
			result[i] = std::fmod(s0[i] + k * alphae[i], 1.0);
		}
		k++;
		return result;
	}
	void setOffset(valarray<double> s) { s0 = s; }
	void setIteration(size_t i) { k = i; }
};


valarray<valarray<double>> findPath(
	const valarray<double>& start,
	const valarray<valarray<double>>& targets,
	const valarray<double>& min,
	const valarray<double>& max,
	function<bool(const valarray<double>&)> check,
	function<double(const valarray<double>&, const valarray<double>&)> distance,
	function<unsigned int(const valarray<double>&, const valarray<double>&)> subdivider,
	const string& planner,
	const std::map<std::string, std::string>& plannerParams,
	const std::valarray<std::valarray<double>>& waypointSuggestions,
	double jiggle,
	size_t maxChecks,
	double timeout,
	size_t randomSeed,
	bool verbose
) {

	const size_t n = start.size();
	const size_t numTargets = targets.size();

	if (min.size() != n || max.size() != n) {
		cout << "start:\n" << start << "\n";
		cout << "min:\n" << min << "\n";
		cout << "max:\n" << max << "\n";
		throw runtime_error("findPath: dimension mismatch");
	}
	for (size_t t = 0; t < numTargets; t++) {
		if (targets[t].size() != n) {
			cout << "start:\n" << start << "\n";
			cout << "targets[" << t << "]:\n" << targets[t] << "\n";
			throw runtime_error("findPath: dimension mismatch");
		}
	}

	auto arrayFromState = [start](const ob::State* state) {
		const auto* rvState = state->as<ob::RealVectorStateSpace::StateType>();
		valarray<double> buffer(start.size());

		for (size_t i = 0; i < buffer.size(); i++) {
			buffer[i] = rvState->values[i];
		}
		return buffer;
	};

	auto space = make_shared<ModdedRealVectorStateSpace>(n, distance, subdivider);

	auto isValid = [&](const ob::State* state) {
		auto values = arrayFromState(state);
		if (verbose) { cout << "checking " << values << ": \n"; }
		auto result = check(values);
		if (verbose) { cout << (result ? "ok" : "invalid") << "\n"; }
		return result;
	};

	ob::ScopedState<> obStart(space);
	vector<ob::ScopedState<>> obTargets;
	obTargets.reserve(numTargets);
	ob::RealVectorBounds bounds(n);

	for (size_t j = 0; j < n; j++) {
		obStart[j] = start[j];
		bounds.setLow(j, min[j]);
		bounds.setHigh(j, max[j]);
	}
	for (size_t t = 0; t < numTargets; t++) {
		obTargets.push_back(ob::ScopedState<>(space));
		for (size_t j = 0; j < n; j++) {
			obTargets[t][j] = targets[t][j];
		}
	}

	// in case a configuration is on the edge of being valid, try to jiggle it free
	auto applyJiggle = [isValid, n, jiggle](ob::ScopedState<>& state) {
		if (isValid(state.get())) { return true; }
		else if (jiggle == 0) { return false; }
		else {
			for (size_t j = 0; j < n; j++) {
				for (double delta: {jiggle, -jiggle}) {

					state[j] += delta;
					if (isValid(state.get())) {
						return true;
					}
					state[j] -= delta;
				}
			}
		}
		return false;
	};

	space->setBounds(bounds);

	og::SimpleSetup ss(space);

	ss.setStateValidityChecker(isValid);

	if (plannerFactories.count(planner) == 0) {
		cout << "no planner named \"" << planner << "\"; "
			 << "available planners: ";
		for (const auto& p: plannerFactories) {
			cout << '"' << p.first << "\" ";
		}
		cout << "\n";
	}

	ss.setPlanner(plannerFactories.at(planner)(ss.getSpaceInformation()));

	bool paramProblem = false;
	for (const auto& [p, v]: plannerParams) {
		if (not ss.getPlanner()->params().hasParam(p)) {
			cout << "planner " << planner << " has no parameter named \"" << p << "\"\n";
			paramProblem = true;
		}
	}

	if (paramProblem) {
		cout << "planner configuration:" << "\n";
		ss.getPlanner()->params().print(cout);
		throw runtime_error("invalid planner parameters");
	}

	bool success = ss.getPlanner()->params().setParams(plannerParams);

	if (not success) {
		throw runtime_error("error while parameterizing planner");
	}

	auto randomizer = make_shared<AdditiveRecurrenceSequence>(n);
	randomizer->setIteration(randomSeed * 1000000);

	space->setStateSamplerAllocator(
		[n, &randomizer, &waypointSuggestions](const ob::StateSpace* space) {
			return make_shared<
				BiasedSampler<ob::RealVectorDeterministicStateSampler>
				>(space, waypointSuggestions, 15, randomizer);
		});

	bool startOk = applyJiggle(obStart);
	if (!startOk) {
		cout << "start configuration invalid, aborting" << "\n";
		return {};
	}
	ss.setStartState(obStart);

	cout << "checking targets..." << "\n";

	auto goals = make_shared<ob::GoalStates>(ss.getSpaceInformation());
	int validTarget = -1;
	for (auto&& [i, obTarget]: enumerate(obTargets)) {
		bool targetOk = applyJiggle(obTarget);
		if (targetOk) {
			goals->addState(obTarget);
			validTarget = i;
		}
	}
	if (goals->getStateCount() == 0) {
		cout << "no valid target configuration, aborting" << "\n";
		return {};
	}
	else if (goals->getStateCount() == 1) {
		ss.setGoalState(obTargets[validTarget]);
	}
	else {
		ss.setGoal(goals);
	}

	ss.setup();

	if (verbose) {
		ss.print();
	}

	cout << "direct path check..." << "\n";

	valarray<valarray<double>> result;

	// check if direct motion to the closest target is possible
	ob::ScopedState<> closestTarget(space);
	double closestDistance = std::numeric_limits<double>::infinity();
	for (const auto& target: obTargets) {
		double d = distance(
			arrayFromState(&*obStart), arrayFromState(&*target));
		if (d < closestDistance) {
			closestDistance = d;
			closestTarget = target;
		}
	}

	if (ss.getSpaceInformation()->getMotionValidator()->checkMotion(
			&*obStart, &*closestTarget)) {

		result = {
			arrayFromState(&*obStart),
			arrayFromState(&*closestTarget)
		};
	}
	else {

		auto ptcIter = ob::IterationTerminationCondition(maxChecks);
		auto ptcTime = ob::timedPlannerTerminationCondition(timeout);
		auto ptcBoth = ob::plannerOrTerminationCondition(ptcIter, ptcTime);

		ob::PlannerTerminationCondition ptc =
			(timeout <= 0 || isnan(timeout) || isinf(timeout)) ?
			ptcIter : ptcBoth;

		cout << "solving..." << "\n";

		ob::PlannerStatus solved = ss.solve(ptc);

		if (solved && ss.haveExactSolutionPath()) {
			auto& states = ss.getSolutionPath().getStates();

			cout << "solved, " << states.size() << " waypoints\n";

			result.resize(states.size());
			for (size_t i = 0; i < states.size(); i++) {
				result[i] = arrayFromState(states[i]);
			}
		}
		else {
			cout << "no solution found" << "\n";
		}

	}

	return result;
}
