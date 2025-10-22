
#include "planner_headers.h"

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose,
	x, y, z, qx, qy, qz, qw);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseUpdate,
	x, y, z, qx, qy, qz, qw);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Collision,
	step, link1_id, link2_id, position);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlannerParamInfo,
	defaultValue, rangeSuggestion);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlannerInfo,
	multithreaded, directed, params);

json json_call_impl(
	GestaltPlanner& self,
	const json& data
) {

	// try {

	assert(data.at("jsonrpc").get<string>() == "2.0");
	string method = data.at("method");
	json params = data.contains("params") ? data.at("params") : json::array();
	int id = data.at("id");
	json result = nullptr;

#include "jsondispatch.inc"

	// }
	// catch (runtime_error& e) {
	// 	return {
	// 		{"jsonrpc", "2.0"},
	// 		{"id", nullptr},
	// 		{"error", {
	// 			{"code", -1},
	// 			{"message", e.what()}
	// 		}}
	// 	};
	// }

	return {};
}

/*( public: )*/ string GestaltPlanner::json_call(
	const string& data
) {
	auto guard = state->log.log("gp.json_call",
		data);

	auto j = json::parse(data);

	if (j.is_object()) {
		return json_call_impl(*this, j).dump();
	}
	else if (j.is_array()) {
		auto results = json::array();
		for (const auto& request : j) {
			results.push_back(json_call_impl(*this, request));
		}
		return results.dump();
	}
	else {
		throw runtime_error("invalid json_call parameter");
	}
}
