
#pragma once

#include "common.h"
#include <Eigen/Dense>

inline Eigen::Map<Eigen::VectorXd> eigWrap(valarray<double>& v) {
	return Eigen::Map<Eigen::VectorXd>(&v[0], v.size());
}

inline const Eigen::Map<const Eigen::VectorXd> eigWrap(const valarray<double>& v) {
	return Eigen::Map<const Eigen::VectorXd>(&v[0], v.size());
}

// use Eigen::VectorXd ev = eigWrap(v) to copy
// use auto ev = eigWrap(v) to reference (and modify v)

class Subspace {
	Eigen::MatrixXd eigConstraints;
	Eigen::VectorXd eigSupport;
	Eigen::MatrixXd nullSpace;
public:
	const size_t superDims = 0;
	const size_t subDims = 0;

	Subspace(
		const valarray<double>& support,
		const valarray<valarray<double>>& constraints
	) :
		superDims{ support.size() },
		subDims{ superDims - constraints.size() }
	{
		assert(subDims > 0);

		if (constraints.size() == 0) {
			return;
		}

		eigConstraints = Eigen::MatrixXd(constraints.size(), superDims);

		for (size_t c = 0; c < constraints.size(); c++) {
			assert(constraints[c].size() == superDims);
			for (size_t j = 0; j < superDims; j++) {
				eigConstraints(c, j) = constraints[c][j];
			}
		}

		eigSupport = eigWrap(support);
		nullSpace = Eigen::FullPivLU<Eigen::MatrixXd>(eigConstraints).kernel();
		// std::cout << "support\n" << eigSupport << "\n";
		// std::cout << "nullSpace\n" << nullSpace << "\n";
	}

	valarray<double> project(
		const valarray<double>& params,
		bool derivative = false,
		double tolerance = 1e-6
	) {

		if (superDims == subDims) {
			return params;
		}

		const auto eigParams = eigWrap(params);
		if (
			(eigConstraints * (eigParams - eigSupport * !derivative))
			.cwiseAbs().maxCoeff() > tolerance
			) {

			cout << "the following parameters do not meet the constraints: " << params  << "\n";
			throw runtime_error("constraint discrepancy");
		}

		valarray<double> result(subDims);

		// only subtract support vector when we are not dealing with derivatives
		eigWrap(result) =
			Eigen::FullPivLU<Eigen::MatrixXd>(nullSpace).solve(
				eigParams - eigSupport * !derivative);

		return result;
	}

	valarray<valarray<double>> project(
		const valarray<valarray<double>>& trajectory,
		bool derivative = false,
		double tolerance = 1e-6
	) {
		if (superDims == subDims) {
			return trajectory;
		}

		valarray<valarray<double>> result(trajectory.size());
		for (size_t i = 0; i < trajectory.size(); i++) {
			result[i] = project(trajectory[i], derivative, tolerance);
		}
		return result;
	}

	valarray<double> unproject(
		const valarray<double>& params,
		bool derivative = false
	) {
		if (superDims == subDims) {
			return params;
		}

		const auto eigParams = eigWrap(params);

		valarray<double> result(superDims);

		// only add support vector when we are not dealing with derivatives
		eigWrap(result) = !derivative * eigSupport + nullSpace * eigParams;

		return result;
	}

	valarray<valarray<double>> unproject(
		const valarray<valarray<double>>& trajectory,
		bool derivative = false
	) {
		if (superDims == subDims) {
			return trajectory;
		}

		valarray<valarray<double>> result(trajectory.size());
		for (size_t i = 0; i < trajectory.size(); i++) {
			result[i] = unproject(trajectory[i], derivative);
		}
		return result;
	}
};