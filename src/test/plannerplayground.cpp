#include <iostream>
#include <string>
#include <stl.h>

// #define CONTAINER_CAST_BULLET
#include "btBulletCollisionCommon.h"
#include <Eigen/Core>
#include "../planner/utils/containertools.hpp"

using namespace std;

struct O {
	const int id;
};

template<typename T>
T make_from_json(const nlohmann::json&);

template<>
O make_from_json<O>(const nlohmann::json& j){
	return {j.at("id").get<int>()};
}

int main() {

	json j = "{\"id\":17}";

	auto o=make_from_json<O>(j);

	std::cout << o.id << "\n";

	return 0;
}
