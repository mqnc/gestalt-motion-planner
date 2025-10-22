
#include "../neldermead.hpp"
#include "encapsulate.h"
#include "probedirections.inc"

array<double, 2> projectedLimits(
	Eigen::Vector3d direction,
	const vector<Eigen::Vector3d>& points
) {
	// project all points onto a line
	// and find the furthest extensions

	direction.normalize();
	double min = std::numeric_limits<double>::infinity();
	double max = -std::numeric_limits<double>::infinity();

	for (const auto& p : points) {
		auto proj = direction.dot(p);
		if (proj < min) { min = proj; }
		if (proj > max) { max = proj; }
	}
	return { min, max };
}

auto radiusAndVolume(
	const valarray<double>& params,
	const vector<Eigen::Vector3d>& points
) {
	// find the minimum radius and the volume
	// for a capsule (defined by support points encoded in params)
	// so that it encapsulates all points

	const double eps = 1e-9;
	bool degenerate = false;

	Eigen::Vector3d s0(&params[0]);
	Eigen::Vector3d s1(&params[3]);

	Eigen::Vector3d s0s1 = s1 - s0;
	double s2 = s0s1.dot(s0s1);
	double s = sqrt(s2); // distance between support points

	if (s2 < eps * eps) { degenerate = true; } // capsule is a sphere

	double maxDistance = 0;
	for (const auto& p : points) {
		Eigen::Vector3d s0p = p - s0;
		if (degenerate) {
			double r = s0p.norm();
			if (r > maxDistance) { maxDistance = r; }
		}
		else {
			double projection = s0p.dot(s0s1) / s2;
			double r =
				projection <= 0 ? s0p.norm() :
				projection >= 1 ? (p - s1).norm() :
				s0p.cross(s0s1).norm() / s;
			if (r > maxDistance) { maxDistance = r; }
		}
	}

	double r = maxDistance;
	double volume = M_PI * r * r * s + 4.0 / 3.0 * M_PI * r * r * r;

	return std::make_pair(r, volume);
};


Capsule encapsulatePoints(vector<valarray<double>>& points) {

	// convert input for use with Eigen
	vector<Eigen::Vector3d> eigPoints(points.size());
	for (size_t i = 0; i < points.size(); i++) {
		eigPoints[i] = Eigen::Vector3d(&points[i][0]);
	}

	// find direction along which points are stretched out furthest
	double max = -std::numeric_limits<double>::infinity();
	Eigen::Vector3d dirMax{ 0, 0, 0 };
	array<double, 2> limitsMax{ 0, 0 };
	for (const auto& dir : probeDirections) {
		auto limits = projectedLimits(dir, eigPoints);
		double l = limits[1] - limits[0];
		if (l > max) {
			max = l;
			dirMax = dir;
			limitsMax = limits;
		}
	}

	// find projection limits along 2 further orthogonal directions
	Eigen::Vector3d other = Eigen::Vector3d(1, 0.001, 0);
	Eigen::Vector3d ortho1 = dirMax.cross(other).normalized();
	Eigen::Vector3d ortho2 = dirMax.cross(ortho1).normalized();
	auto limitsOrtho1 = projectedLimits(ortho1, eigPoints);
	auto limitsOrtho2 = projectedLimits(ortho2, eigPoints);

	// find approximate capsule radius
	double r = std::max(
		(limitsOrtho1[1] - limitsOrtho1[0]) / 2,
		(limitsOrtho2[1] - limitsOrtho2[0]) / 2
	);

	// compute approximate support points
	Eigen::Vector3d s1 =
		(limitsMax[0] + r) * dirMax
		+ 0.5 * (limitsOrtho1[0] + limitsOrtho1[1]) * ortho1
		+ 0.5 * (limitsOrtho2[0] + limitsOrtho2[1]) * ortho2;
	Eigen::Vector3d s2 =
		(limitsMax[1] - r) * dirMax
		+ 0.5 * (limitsOrtho1[0] + limitsOrtho1[1]) * ortho1
		+ 0.5 * (limitsOrtho2[0] + limitsOrtho2[1]) * ortho2;

	// cost function for optimization
	auto cost = [&eigPoints](const valarray<double>& params) {
		auto rad_vol = radiusAndVolume(params, eigPoints);
		return rad_vol.second;
	};

	// optimize approximate support points
	valarray<double> start = { s1[0], s1[1], s1[2], s2[0], s2[1], s2[2] };

	//auto rad_vol0 = radiusAndVolume(start, eigPoints);
	//cout << rad_vol0.second << "\n";

	NelderMead nm(cost, start);
	auto result = nm.steps(200).args;

	auto rad_vol = radiusAndVolume(result, eigPoints);
	//cout << rad_vol.second << "\n";

	return {
		{result[0], result[1], result[2]},
		{result[3], result[4], result[5]},
		rad_vol.first
	};
}