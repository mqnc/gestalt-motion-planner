#include "multibody.h"

// void updateBulletTrafo(btTransform& toBeUpdated, const btTransform& update) {
// 	const double x = update.getOrigin().getX();
// 	const double y = update.getOrigin().getY();
// 	const double z = update.getOrigin().getZ();
// 	const auto& basis = update.getBasis();
// 	if (!isnan(x)) { toBeUpdated.getOrigin().setX(x); }
// 	if (!isnan(y)) { toBeUpdated.getOrigin().setY(y); }
// 	if (!isnan(z)) { toBeUpdated.getOrigin().setZ(z); }
// 	if (!isnan(basis[0][0])) {
// 		toBeUpdated.setBasis(basis);
// 	}
// }

// void transformLinkHierarchy(
// 	BulletJoint* joint,
// 	const btTransform& worldToParent
// ) {
// 	btTransform trafo = worldToParent * joint->getCurrentLocalTrafo();
// 	auto& link = joint->child;
// 	link->bulletObject.setWorldTransform(trafo);
// 	for (const auto& childJoint : link->children) {
// 		transformLinkHierarchy(childJoint, trafo);
// 	}
// }
