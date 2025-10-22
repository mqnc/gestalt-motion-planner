#pragma once

#include "common.h"

#include "bullet.hpp"
#include "shapes.hpp"

#include "../collision/collisionrobot.h"

namespace nlohmann {
	template <>
	struct adl_serializer<BulletLink> {

		static BulletLink from_json(const json& j) {
			BulletLink result({
				j.at("id").get<string>(),
				j.at("collisionShape").get<Holder<Shape>>(),
				j.at("visualShape").get<Holder<Shape>>()
				});

			return result;
		}

		static void to_json(json& j, const BulletLink& bl) {
			j = {
				{"id", bl.id},
				{"collisionShape", bl.collisionShape},
				{"visualShape", bl.visualShape},
			};
		}
	};
}

namespace nlohmann {
	template <>
	struct adl_serializer<BulletLinkTemplate> {

		static BulletLinkTemplate from_json(const json& j) {
			return {
				j.at("id").get<string>(),
				j.at("collisionShape").get<Holder<Shape>>(),
				j.at("visualShape").get<Holder<Shape>>()
			};
		}

		static void to_json(json& j, const BulletLinkTemplate& bl) {
			j = {
				{"id", bl.id},
				{"collisionShape", bl.collisionShape},
				{"visualShape", bl.visualShape}
			};
		}
	};
}

NLOHMANN_JSON_SERIALIZE_ENUM(JointType, {
	{JointType::floating, "floating"},
	{JointType::fixed, "fixed"},
	{JointType::revolute, "revolute"},
	{JointType::continuous, "continuous"},
	{JointType::prismatic, "prismatic"}
	});


namespace nlohmann {
	template <>
	struct adl_serializer<BulletJoint> {

		static BulletJoint from_json(const json& j) {

			auto bj = BulletJoint(
				j.at("id").get<string>(),
				j.at("type").get<JointType>(),
				j.at("origin").get<btTransform>(),
				j.at("axis").get<btVector3>(),
				{
					j.at("limits")[0].get<double>(),
					j.at("limits")[1].get<double>()
				}
			);

			j.at("parentLink").get_to(bj.parentId);
			j.at("childLink").get_to(bj.childId);

			if (
				bj.type == JointType::revolute
				|| bj.type == JointType::continuous
				|| bj.type == JointType::prismatic
				) {
				bj.set(j.at("currentValue").get<double>());
			}
			else if (bj.type == JointType::floating) {
				bj.set(j.at("currentLocalTrafo").get<btTransform>());
			}

			return bj;
		}

		static void to_json(json& j, const BulletJoint& bj) {
			j = {
				{"id", bj.id},
				{"type", bj.type},
				{"origin", bj.origin},
				{"axis", bj.axis},
				{"limits", bj.limits},
				{"parentLink", bj.parentId},
				{"childLink", bj.childId},
				{"currentValue", bj.getCurrentJointValue()},
				{"currentLocalTrafo", bj.getCurrentLocalTrafo()}
			};
		}

	};
}


class CollisionRobotSerializer {
public:
	static void to_json(json& j,
		const shared_ptr<CollisionRobotTemplate>& tmpl) {
		j = {
			{"id", tmpl->id},
			{"linkTemplates", tmpl->linkTemplates},
			{"jointTemplates", tmpl->jointTemplates},
			{"selectedJoints", tmpl->selectedJoints},
			{"collisionIgnoreGroups", tmpl->collisionIgnoreGroups}
		};
	}

	static void to_json(json& j, const CollisionRobot& robot) {
		j = {
			{"id", robot.id},
			{"parent", nullptr},
			{"parentLink", nullptr},
			{"links", robot.links},
			{"joints", robot.joints},
			{"preSelectedJoints", robot.preSelectedJoints},
			{"selectedJoints", robot.selectedJoints},
			{"collisionIgnoreGroups", robot.collisionIgnoreGroups}
		};
		if (robot.parent) {
			j["parent"] = robot.parent->id;
		}
		if (robot.parentLink) {
			j["parentLink"] = robot.parentLink->id;
		}
	}

	template<typename T>
	static void get_to(const json& j, const string& field, T& target) {
		target = j.at(field).get<T>();
	}

	static void from_json(const json& j, shared_ptr<CollisionRobotTemplate>& tmpl) {
		tmpl = make_shared<CollisionRobotTemplate>();
		for (const auto& [id, lt] : j.at("linkTemplates").items()) {
			tmpl->linkTemplates.try_emplace(id,
				lt.at("id").get<string>(),
				lt.at("collisionShape").get<Holder<Shape>>(),
				lt.at("visualShape").get<Holder<Shape>>()
			);
		}
		get_to(j, "jointTemplates", tmpl->jointTemplates);
		get_to(j, "selectedJoints", tmpl->selectedJoints);
		get_to(j, "collisionIgnoreGroups", tmpl->collisionIgnoreGroups);
	}

	static void from_json(const json& j, CollisionRobot& robot) {
		get_to(j, "id", robot.id);
		get_to(j, "links", robot.links);
		get_to(j, "joints", robot.joints);
		get_to(j, "preSelectedJoints", robot.preSelectedJoints);
		get_to(j, "selectedJoints", robot.selectedJoints);
		get_to(j, "collisionIgnoreGroups", robot.collisionIgnoreGroups);

		robot.init();
	}
};

inline void to_json(json& j, const shared_ptr<CollisionRobotTemplate>& tmpl) {
	CollisionRobotSerializer::to_json(j, tmpl);
}

inline void to_json(json& j, const CollisionRobot& robot) {
	CollisionRobotSerializer::to_json(j, robot);
}

inline void from_json(const json& j, shared_ptr<CollisionRobotTemplate>& tmpl) {
	CollisionRobotSerializer::from_json(j, tmpl);
}

inline void from_json(const json& j, CollisionRobot& robot) {
	CollisionRobotSerializer::from_json(j, robot);
}