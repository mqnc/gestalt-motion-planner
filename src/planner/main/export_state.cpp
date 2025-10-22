
#include "planner_headers.h"


NLOHMANN_JSON_SERIALIZE_ENUM(JointType, {
		{JointType::undefined, "undefined"},
		{JointType::floating, "floating"},
		{JointType::fixed, "fixed"},
		{JointType::revolute, "revolute"},
		{JointType::continuous, "continuous"},
		{JointType::prismatic, "prismatic"}
	})

inline void to_json(json& j, const btTransform& t) {
	btVector3 origin = t.getOrigin();
	btQuaternion rotation = t.getRotation();
	j = json {
		{"x", origin.getX()},
		{"y", origin.getY()},
		{"z", origin.getZ()},
		{"qx", rotation.getX()},
		{"qy", rotation.getY()},
		{"qz", rotation.getZ()},
		{"qw", rotation.getW()}
	};
}

inline void to_json(json& j, const Part& p) {
	j = json {};
	j["joint"] = json::object();
	j["joint"]["id"] = p.jointId;
	j["joint"]["type"] = p.type;
	j["joint"]["axis"] = {p.axis.getX(), p.axis.getY(), p.axis.getZ()};
	j["joint"]["value"] = p.getCurrentJointValue();
	j["joint"]["limits"] = {p.limits[0], p.limits[1]};
	j["joint"]["origin"] = p.origin;
	if (p.parent) {
		j["parent"] = p.parent->robotId + "." + p.parent->linkId;
	}
	else {
		j["parent"] = json {};
	}
	j["currentLocalTrafo"] = p.getCurrentLocalTrafo();
	j["currentGlobalTrafo"] = p.bulletObject.getWorldTransform();
	j["safetyMargin"] = p.collisionShape->getSafetyMargin();
};

inline void to_json(json& j, const CollisionRobot& r) {
	j = json {};
	auto parent = r.getParent();
	string part1 = parent.first == nullptr ? "" : parent.first->getId();
	string part2 = parent.second == nullptr ? "" : parent.second->linkId;
	j["parent"] = part1 + "." + part2;
	if (j["parent"] == ".") {
		j["parent"] = json {};
	}
	j["parts"] = json::object();
	for (const auto& part: r.getParts()) {
		j["parts"][part->linkId] = part;
	}
	j["selectedJoints"] = json::array();
	for (size_t i = 0; i < r.getJointSelection().size(); i++) {
		j["selectedJoints"].push_back({
			{"id", r.getJointSelection()[i]},
			{"value", r.getJointPositions()[i]}
		});
	}
	j["collisionIgnoreGroups"] = r.getCollisionIgnoreGroups();
};

inline void to_json(json& j, const PlannerState& s) {
	j = json {};
	j["robots"] = s.robots;
	j["collisionIgnoreGroups"] = s.collisionIgnoreGroupManager.getGroups();
	vector<string> cache;
	for (auto it = s.getCache().cbegin(); it != s.getCache().cend(); ++it) {
		cache.push_back(it->first);
	}
	j["fileCache"] = cache;
}

// leave object_ids empty to export entire scene
/*( public: )*/ string GestaltPlanner::export_state(
	string object_id /*( = "" )*/,
	bool indent /*( = true )*/
) {
	auto guard = state->log.log("gp.export_state", object_id, indent);

	if (object_id == "") {
		if (indent) {
			return json(*state).dump(1, '\t');
		}
		else {
			return json(*state).dump();
		}
	}
	else {
		if (indent) {
			return json(state->getRobot(object_id)).dump(1, '\t');
		}
		else {
			return json(state->getRobot(object_id)).dump();
		}

	}

}
