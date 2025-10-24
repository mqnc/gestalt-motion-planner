
// https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method#One_possible_variation_of_the_NM_algorithm

#include <valarray>
#include <vector>
#include <functional>
#include <cassert>
#include <iostream>
#include <cmath>

template<typename F>
class NelderMead {

	struct Vertex {
		std::valarray<double> args;
		double cost = std::nan("");
	};

	std::vector<Vertex> simplex;
	const F fn;

	// void print(const std::valarray<double>& v) {
	// 	for (const auto& arg : v) {
	// 		std::cout << arg << " " << std::flush;
	// 	}
	// }

	// void print(const Vertex& v) {
	// 	print(v.args);
	// 	std::cout << " -> " << v.cost << "\n";
	// }

	// void print() {
	// 	for (const auto& v : simplex) {
	// 		print(v);
	// 	}
	// 	std::cout << "\n";
	// }

	inline Vertex makeVertex(const std::valarray<double>& args) {
		Vertex v{ args, fn(args) };
		return v;
	}

public:
	const size_t n; // number of dimensions
	const size_t nv; // number of vertices

	double alpha = 1.0; // reflection parameter
	double gamma = 2.0; // expansion parameter
	double rho = 0.5; // contraction parameter
	double sigma = 0.5; // shrinkage parameter

	NelderMead(F costFn, const std::valarray<double>& startArgs) :
		fn{ costFn },
		n{ startArgs.size() },
		nv{ startArgs.size() + 1 }
	{
		simplex.reserve(nv);
		simplex.push_back(makeVertex(startArgs));
		for (size_t j = 0; j < n; j++) {
			auto args = startArgs;
			args[j] = (args[j] == 0) ? 0.0075 : (args[j] * 1.05);
			simplex.push_back(makeVertex(args));
		}
	}

	NelderMead(F costFn, const std::vector<std::valarray<double>>& startSimplex) :
		fn{ costFn },
		nv{ startSimplex.size() },
		n{ startSimplex.size() - 1 }
	{
		simplex.reserve(nv);
		for (const auto& args : startSimplex) {
			assert(args.size() == n);
			simplex.push_back(makeVertex(args));
		}
	}

	// call in case you continue after changing the cost function
	void refreshAllCosts() {
		for (size_t i = 0; i < nv; i++) {
			simplex[i] = makeVertex(simplex[i].args);
		}
	}

	Vertex step() {

		// 1 - sort
		std::sort(simplex.begin(), simplex.end(),
			[](const Vertex& v1, const Vertex& v2) {
				return (v1.cost < v2.cost);
			}
		);

		Vertex& best = simplex[0];
		Vertex& worst = simplex[nv - 1];
		Vertex& formerWorst = worst; // for semantic clarification later
		Vertex& secondWorst = simplex[nv - 2];

		// 2 - find centroid of non-worst vertices
		std::valarray<double> centroid(0.0, n);
		for (size_t i = 0; i < nv - 1; i++) {
			centroid += simplex[i].args / (nv - 1);
		}

		// 3 - reflect worst vertex through centroid
		auto reflected = makeVertex(centroid + alpha * (centroid - worst.args));
		if (best.cost <= reflected.cost
			&& reflected.cost < secondWorst.cost) {

			formerWorst = reflected;
			return best;
		}

		// 4 - expansion
		if (reflected.cost < best.cost) {
			auto expanded = makeVertex(centroid + gamma * (reflected.args - centroid));
			if (expanded.cost < reflected.cost) {
				formerWorst = expanded;
				return formerWorst;
			}
			else {
				formerWorst = reflected;
				return formerWorst;
			}
		}

		// 5 - contraction
		auto contracted = makeVertex(centroid + rho * (worst.args - centroid));
		if (contracted.cost < worst.cost) {
			formerWorst = contracted;
			return best;
		}

		// 6 - shrink
		size_t iNewBest = 0;
		for (size_t i = 1; i < nv; i++) {
			simplex[i] = makeVertex(best.args + sigma * (simplex[i].args - best.args));
			if (simplex[i].cost < simplex[iNewBest].cost) {
				iNewBest = i;
			}
		}
		return simplex[iNewBest];
	}

	Vertex steps(size_t num) {
		for (size_t i = 0; i < num - 1; i++) {
			step();
		}
		return step();
	}

};

// int main() {

// 	auto rosenbrock = [](const std::valarray<double>& xy)->double {
// 		double x = xy[0], y = xy[1];
// 		return pow(1 - x, 2) + 100 * pow(y - x * x, 2);
// 	};

// 	std::valarray<double> start = { 4, 5 };
// 	NelderMead nm(rosenbrock, start);

// 	for(size_t i=0; i<120; i++){
// 		auto result = nm.step();
// 		std::cout << "[" << result.args[0] << ", " << result.args[1] << "]\n";
// 	}

// 	return 0;
// }