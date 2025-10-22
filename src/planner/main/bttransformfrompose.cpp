
#include "planner_headers.h"

btTransform btTransformFromPose(const Pose& pose) {
	size_t qnans = isnan(pose.qx) + isnan(pose.qy)
		+ isnan(pose.qz) + isnan(pose.qw);
	if (qnans != 0 && qnans != 4) {
		throw runtime_error("orientation cannot be changed partially");
	}
	return btTransform(
		btQuaternion(pose.qx, pose.qy, pose.qz, pose.qw),
		btVector3(pose.x, pose.y, pose.z)
	);
}
