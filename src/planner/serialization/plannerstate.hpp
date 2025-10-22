#pragma once

#include "common.h"

#include "bullet.hpp"
#include "shapes.hpp"
#include "collisionrobot.hpp"

#include "../main/plannerstate.h"

inline void to_json(json& j, const PlannerState& ps) {
	j = {
		{"robotTemplates", ps.robotTemplates},
		{"robots", ps.robots},
		{"collisionIgnoreGroups", ps.collisionIgnoreGroupManager.getGroups()}
	};
}

inline void from_json(const json& j, PlannerState& ps) {

	j.at("robotTemplates").get_to(ps.robotTemplates);

	j.at("robots").get_to(ps.robots);

	// hook up parents
	for (const auto& [id, robot] : j.at("robots").items()) {
		if (!robot.at("parent").is_null()) {
			ps.robots.at(id).setParent(
				&ps.robots.at(robot.at("parent")),
				robot.at("parentLink")
			);
		}
	}
	transformLinkHierarchy(ps.robots.at("__world__").root, btTransform::getIdentity());

	ps.collisionIgnoreGroupManager.clear();
	for (auto& [id, group] : j.at("collisionIgnoreGroups").items()) {
		ps.collisionIgnoreGroupManager.createGroup(
			id, group.get<vector<string>>());
	}
}
