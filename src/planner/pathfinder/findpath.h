
#pragma once

#include "common.h"

valarray<valarray<double>> findPath(
	const valarray<double>& start,
	const valarray<valarray<double>>& targets,
	const valarray<double>& min,
	const valarray<double>& max,
	function<bool(const valarray<double>&)> check,
	function<double(const valarray<double>&, const valarray<double>&)> distance,
	function<unsigned int(const valarray<double>&, const valarray<double>&)> subdivider,
	const string& planner = "RRTConnect",
	const std::map<std::string, std::string>& plannerParams = {},
	const std::valarray<std::valarray<double>>& waypointSuggestions = {},
	double jiggle = 1e-6,
	size_t maxChecks = 5000,
	double timeout = 5,
	size_t randomSeed = 0,
	bool verbose = false
);
