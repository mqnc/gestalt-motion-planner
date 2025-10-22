
#pragma once

#include "common.h"

#include "btBulletCollisionCommon.h"

#include "collision/shapes.h"
#include "collision/collisionrobot.h"
#include "collision/collisionchecker.h"

#include "meshgenerator.h"
//#include "collisionprobe.hpp"

struct SceneObject {
	string meshHash;
	string meshType;
	bool isActuated = false;
	vector<btTransform> trafo;
};

struct CollisionInfo {
	string link1;
	string link2;
	btVector3 position;
};

struct Scene {
	string title;
	size_t numSteps;
	double dt = nan("");
	dict<SceneObject> objects;
	vector<vector<CollisionInfo>> collisions;
};

inline pair<dict<Mesh>, dict<SceneObject>>
getParts(PlannerState& state) {

	dict<Mesh> meshes;
	dict<SceneObject> links;

	for (const auto& [robotId, robot] : state.robots) {
		for (const auto& part : robot.getParts()) {
			for (const bool isMargin : { false, true }) {

				auto shape =
					isMargin ? part->collisionShape.get()
					: part->visualShape.get();
				
				auto mesh = MeshGenerator(shape).get();

				if (meshes.count(mesh.hash) == 0) {
					meshes[mesh.hash] = mesh;
				}

				links[
					robotId + "."
						+ part->linkId + "."
						+ (isMargin ? "margin" : "hull")] =
					{
						mesh.hash,
						isMargin ? "margin" : "hull",
						false,
						{part->bulletObject.getWorldTransform()}
					};
			}
		}
	}

	return { meshes, links };
}

inline vector<CollisionInfo> getCollisionInfo(PlannerState& state) {

	CollisionChecker checker(state.extractBulletObjectsAndBitMasks());

	auto report = checker.checkCollisions(true);

	vector<CollisionInfo> result;

	std::unordered_map<const btCollisionObject*, string> linkNames;

	for (const auto& [robotId, robot] : state.robots) {
		for (const auto& part : robot.getParts()) {
			linkNames[&part->bulletObject] =
				robotId + "." + part->linkId;
		}
	}

	for (const auto& col : *report.collisions) {
		result.push_back({
			linkNames.at(col.objectA),
			linkNames.at(col.objectB),
			col.pointOnB
			});
	}

	return result;
}

inline vector<pair<string, const Part&>>
listActuatedParts(const PlannerState& state) {

	vector<pair<string, const Part&>> result;

	for (const auto& [robotId, robot] : state.robots) {
		for (const auto& part : robot.getParts()) {
			if (robot.isActuated) {
				result.push_back({
					robotId + "." + part->linkId,
					*part
					});
			}
		}
	}

	return result;
}

inline void animateLinks(
	PlannerState& state,
	const string& object_id,
	const valarray<valarray<double>>& trajectory,
	dict<SceneObject>& out_links,
	vector<vector<CollisionInfo>>& out_collisions
) {
	auto& robot = state.getRobot(object_id);
	size_t n = trajectory.size();

	out_collisions.clear();
	out_collisions.resize(n);

	auto restoreJoints = robot.getJointPositions();
	OnScopeExit jointRestorer(
		[&]() {robot.setJointPositions(restoreJoints);});

	auto active = listActuatedParts(state);

	// mark actuated links
	// and delete their transformations for filling them later
	for (const auto& [name, ref] : active) {
		for (const auto& meshType : { ".margin", ".hull" }) {
			const auto& typedName = name + meshType;
			out_links.at(typedName).isActuated = true;
			out_links.at(typedName).trafo.clear();
			out_links.at(typedName).trafo.reserve(n);
		}
	}

	// perform motion
	for (size_t i = 0; i < n; i++) {
		const auto& q = trajectory[i];
		robot.setJointPositions(q);
		for (const auto& [fullPartId, part] : active) {
			for (const auto& meshType : { ".margin", ".hull" }) {
				const auto& typedName = fullPartId + meshType;
				out_links.at(typedName).trafo.push_back(
					part.bulletObject.getWorldTransform());
			}
		}
		out_collisions[i] = getCollisionInfo(state);
	};
}
