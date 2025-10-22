
#include "stl.h"
#include "btBulletCollisionCommon.h"

double distance(
	const btTransform& t1,
	const btTransform& t2 = btTransform::getIdentity()
) {
	return btDistance(t1.getOrigin(), t2.getOrigin())
		+ t1.getRotation().angleShortestPath(t2.getRotation());
}

bool allNaN(const btTransform& t) {
	const auto& p = t.getOrigin();
	const auto& q = t.getRotation();

	return isnan(p[0]) && isnan(p[1]) && isnan(p[2])
		&& isnan(q[0]) && isnan(q[1]) && isnan(q[2]) && isnan(q[3]);
}

inline std::string to_string(const btTransform& t) {
	const auto& p = t.getOrigin();
	const auto& q = t.getRotation();
	std::stringstream ss;
	ss << "{xyz = " << p[0] << ", " << p[1] << ", " << p[2] << "; ";
	ss << "qxyzw = " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "}";
	return ss.str();
}