#include "planner_headers.h"

/*( public: )*/ valarray<valarray<double>> GestaltPlanner::time_parameterize_path(
	const valarray<valarray<double>> trajectory,
	double dt,
	valarray<double> max_velocity,
	valarray<double> max_acceleration,
	valarray<double> max_jerk
) {
	auto guard = state->log.log("gp.time_parameterize_path",
		trajectory, dt, max_velocity, max_acceleration, max_jerk);

	// q := q(t)
	// f := f(q)
	// x = f
	// vmax >= | q′ f′                  |
	// amax >= | q′²f″           + q″f′ |
	// jmax >= | q′³f‴ + 3q″q′f″ + q‴f′ |

	// first approach: simple constant warp factor w
	// q  = w*t
	// q′ = w
	// q″ = 0
	// q‴ = 0
	// x  = f
	// vmax >= w |f′|  ->  vmax/|f′| >= w
	// amax >= w²|f″|  ->  amax/|f″| >= w²
	// jmax >= w³|f‴|  ->  jmax/|f‴| >= w³

	// we assume dt = 1.0 in the given trajectory
	auto pos = trajectory;
	auto vel = diff(pos);
	auto acc = diff(vel);
	auto jrk = diff(acc);

	double w = inf;
	for (const auto& vs : vel) {
		for (auto&& [v, vmax] : zip(vs, max_velocity)) {
			w = std::min(w, vmax / fabs(v));
		}
	}

	double w2 = inf;
	for (const auto& as : acc) {
		for (auto&& [a, amax] : zip(as, max_acceleration)) {
			w2 = std::min(w2, amax / fabs(a));
		}
	}

	double w3 = inf;
	for (const auto& js : jrk) {
		for (auto&& [j, jmax] : zip(js, max_jerk)) {
			w3 = std::min(w3, jmax / fabs(j));
		}
	}

	double warp = min({ w, sqrt(w2), cbrt(w3) });
	double dq = warp * dt;

	size_t numVertices = ceil(trajectory.size() / dq) + 1; // one more vertex than steps (fence post)
	valarray<valarray<double>> result(numVertices);

	double stepSize = double(trajectory.size() - 1) / double(numVertices - 1);
	for (size_t k = 0; k + 1 < numVertices; k++) {
		double q = k * stepSize;
		size_t before = floor(q);
		size_t after = before + 1;
		if(before >= trajectory.size() || after >= trajectory.size()) {
			throw runtime_error("time_parameterize_path: out of bounds");
		}
		double frac = q - before;
		result[k] =
			(1 - frac) * trajectory[before]
			+ frac * trajectory[after];
	}
	result[result.size() - 1] = trajectory[trajectory.size() - 1];

	return result;

}