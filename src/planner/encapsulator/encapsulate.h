
#pragma once

#include "common.h"

#include <Eigen/Dense>

struct Capsule {
	valarray<double> p0{ 0, 0, 0 };
	valarray<double> p1{ 0, 0, 0 };
	double radius;
};

array<double, 2> projectedLimits(
	Eigen::Vector3d direction,
	const vector<Eigen::Vector3d>& points
);

auto radiusAndVolume(
	const Eigen::VectorXd& params,
	const vector<Eigen::Vector3d>& points
);

Capsule encapsulatePoints(vector<valarray<double>>& points);