
#pragma once

#include "common.h"
#include "utils.h"
#include "zip.h"
#include "meshgenerator.h"
#include "sceneexport.hpp"

inline string toJson(double number) {
	return isnan(number) ? "null" : to_string(number);
}

inline string toJson(const btTransform& trafo) {
	stringstream result;
	result << std::fixed << std::setprecision(4);
	const auto& xyz = trafo.getOrigin();
	const auto& qxyzw = trafo.getRotation();
	result << "["
		<< toJson(xyz.x()) << ","
		<< toJson(xyz.y()) << ","
		<< toJson(xyz.z()) << ","
		<< toJson(qxyzw.x()) << ","
		<< toJson(qxyzw.y()) << ","
		<< toJson(qxyzw.z()) << ","
		<< toJson(qxyzw.w()) << "]";
	return result.str();
}

inline string toJson(const Mesh& mesh) {
	stringstream ss;
	ss << std::fixed << std::setprecision(4);

	ss << "{";

	ss << "\"vertices\":[";
	for (auto&& [sep, v] : commasep(mesh.vertices)) {
		ss << sep << "["
			<< toJson(v[0]) << ","
			<< toJson(v[1]) << ","
			<< toJson(v[2]) << "]";
	}
	ss << "]";

	ss << ",";

	ss << "\"faces\":[";
	for (auto&& [sep, f] : commasep(mesh.faces)) {
		ss << sep << "["
			<< f[0] << ","
			<< f[1] << ","
			<< f[2] << "]";
	}
	ss << "]";

	ss << "}";

	return ss.str();
}

inline string toJson(const SceneObject& object) {
	stringstream result;

	result << "{\"mesh\":\"" << object.meshHash << "\","
		<< "\"meshType\":\"" << object.meshType << "\","
		<< "\"isActuated\":" << (object.isActuated ? "true" : "false") << ","
		<< "\"xyz_qxyzw\":";

	if (!object.isActuated) {
		result << toJson(object.trafo[0]);
	}
	else {
		result << "[";
		for (auto&& [sep, trafo] : commasep(object.trafo)) {
			result << sep << toJson(trafo);
		}
		result << "]";
	}
	result << "}";

	return result.str();
}

inline string toJson(const vector<CollisionInfo>& collisions) {
	stringstream result;
	result << "[";
	for (auto&& [sep, col] : commasep(collisions)) {
		result << sep
			<< "{\"obj1\":\"" << col.link1 << "\","
			<< "\"obj2\":\"" << col.link2 << "\","
			<< "\"xyz\":["
			<< toJson(col.position.x()) << ","
			<< toJson(col.position.y()) << ","
			<< toJson(col.position.z()) << "]}";
	}
	result << "]";

	return result.str();
}

inline string toJson(const Scene& scene) {
	stringstream result;
	result << "{";
	{
		result << "\"title\":\"" << scene.title << "\","
			<< "\"numSteps\":" << scene.numSteps << ","
			<< "\"dt\":" << toJson(scene.dt) << ","
			<< "\"objects\":{";
		{
			for (auto&& [sep, name, data] : commasep(scene.objects)) {
				result << sep << "\"" << name << "\":" << toJson(data);
			}
		}
		result << "}";

		result << ",";

		result << "\"collisions\":{";
		{
			string sep = "";
			for (auto&& [frame, frameCollisions] : enumerate(scene.collisions)) {
				if (frameCollisions.size() > 0) {
					result << sep << "\"" << frame << "\":" << toJson(frameCollisions);
					sep = ",";
				}
			}
		}
		result << "}";
	}
	result << "}";

	return result.str();
}

inline string toJson(
	const dict<Mesh>& meshes,
	const vector<Scene>& scenes
) {
	stringstream result;
	result << "{\n";
	{
		result << "\"meshes\":{";
		{
			for (auto&& [sep, hash, mesh] : commasep(meshes)) {
				result << sep << "\"" << hash << "\":" << toJson(mesh);
			}
		}
		result << "},\n";

		result << "\"scenes\":[";
		{
			for (auto&& [sep, scene] : commasep(scenes)) {
				result << sep << toJson(scene);
			}
		}
		result << "]\n";
	}
	result << "}\n";
	return result.str();
}

