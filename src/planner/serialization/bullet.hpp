

#pragma once

#include "common.h"
#include "btBulletCollisionCommon.h"

inline void to_json(json& j, const btVector3& v) {
	j = json{ v[0], v[1], v[2] };
}

inline void from_json(const json& j, btVector3& v) {
	v.setValue(j[0], j[1], j[2]);
}

inline void to_json(json& j, const btQuaternion& q) {
	j = json{ q.x(), q.y(), q.z(), q.w() };
}

inline void from_json(const json& j, btQuaternion& q) {
	q.setValue(j[0], j[1], j[2], j[3]);
}

inline void to_json(json& j, const btTransform& tf) {
	j = json{
		{"xyz", tf.getOrigin()},
		{"qxyzw", tf.getRotation()}
	};
}

inline void from_json(const json& j, btTransform& tf) {
	tf.setOrigin(j.at("xyz").get<btVector3>());
	tf.setRotation(j.at("qxyzw").get<btQuaternion>());
}
