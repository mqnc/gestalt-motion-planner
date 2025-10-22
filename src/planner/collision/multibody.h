
#pragma once

#include "common.h"
#include "shapes.h"

#include "btBulletCollisionCommon.h"

inline void updateBulletTrafo(btTransform& toBeUpdated, const btTransform& update) {
	const double x = update.getOrigin().getX();
	const double y = update.getOrigin().getY();
	const double z = update.getOrigin().getZ();
	const auto& basis = update.getBasis();
	if (!isnan(x)) { toBeUpdated.getOrigin().setX(x); }
	if (!isnan(y)) { toBeUpdated.getOrigin().setY(y); }
	if (!isnan(z)) { toBeUpdated.getOrigin().setZ(z); }
	if (!isnan(basis[0][0])) {
		toBeUpdated.setBasis(basis);
	}
}

enum class JointType {
	undefined, floating, fixed, revolute, continuous, prismatic
};
inline JointType jointTypeFromString(const string& type) {
	if (type == "floating") {
		return JointType::floating;
	}
	else if (type == "fixed") {
		return JointType::fixed;
	}
	else if (type == "revolute") {
		return JointType::revolute;
	}
	else if (type == "continuous") {
		return JointType::continuous;
	}
	else if (type == "prismatic") {
		return JointType::prismatic;
	}
	else {
		throw runtime_error(type + " joint not supported");
	}
}

class PartDescriptor {
public:
	const string jointId = "";
	const string linkId = "";

	JointType type = JointType::undefined;

	btTransform origin = btTransform::getIdentity();
	btVector3 axis = { 0, 0, 0 };
	array<double, 2> limits{ NaN, NaN };

	string parentLinkId = "";

	Holder<Shape> collisionShape;
	Holder<Shape> visualShape;
};

class Part : public PartDescriptor {
public:

	// because parts point to each other, they cannot be copied or moved around
	Part(const Part& other) = delete;
	Part(Part&& other) = delete;

	string robotId = "";
	Part* parent = nullptr;
	unordered_set<Part*> children;

	btCollisionObject bulletObject;

	double currentJointValue = NaN;
	double startJointValue = NaN;
	double targetJointValue = NaN;

	btTransform currentLocalTrafo = btTransform::getIdentity();

	BitMask collisionBitMask = 0;

	Part(const PartDescriptor& descriptor, const string& robotId) :
		PartDescriptor{ descriptor }, robotId{ robotId } {

		// validate parameters
		if (type == JointType::revolute
			|| type == JointType::prismatic) {
			assert(!isnan(limits[0]) && !isnan(limits[1]));
		}
		else {
			assert(isnan(limits[0]) && isnan(limits[1]));
		}
		if (type == JointType::revolute
			|| type == JointType::continuous
			|| type == JointType::prismatic) {
			assert(abs(axis.length() - 1.0) < 1.0e-6);
		}
		else {
			assert(axis.isZero());
		}
		if (type == JointType::floating) {
			// use set() for setting the trafo
			assert(origin == btTransform::getIdentity());
		}

		// initialize
		if (type == JointType::fixed) {
			currentLocalTrafo = origin;
		}
		else if (type == JointType::revolute
			|| type == JointType::prismatic) {
			setJointValue(std::clamp(0.0, limits[0], limits[1]));
		}
		else if (type == JointType::continuous) {
			setJointValue(0.0);
		}

		bulletObject.setCollisionShape(
			collisionShape->getBulletShape());
	}

	void setParent(Part* newParent) {
		if (parent) {
			parent->children.erase(this);
			parent->bulletObject.setIgnoreCollisionCheck(
				&bulletObject, false
			);
		}
		if (newParent) {
			newParent->children.insert(this);
			newParent->bulletObject.setIgnoreCollisionCheck(
				&bulletObject, true
			);
		}
		parent = newParent;
		updateWorldTrafos();
	}

	void setJointValue(double value) {
		if ((type == JointType::revolute || type == JointType::prismatic)
			&& (value<limits[0] || value>limits[1])) {

			stringstream ss;
			ss << "joint value out of limits (" << jointId << "): "
				<< limits[0] << " ≤ " << value << " ≤ " << limits[1] << " violated";

			throw runtime_error(ss.str());
		}

		switch (type) {
		case JointType::revolute: case JointType::continuous:
			currentLocalTrafo = origin * btTransform(btQuaternion(axis, value));
			break;
		case JointType::prismatic:
			currentLocalTrafo = origin * btTransform(
				btMatrix3x3::getIdentity(),
				value * axis
			);
			break;
		default:
			throw runtime_error("joint type cannot be set from scalar value");
		}
		currentJointValue = value;
	}

	void setJointTrafo(const btTransform& trafo) {
		if (type != JointType::floating) {
			throw runtime_error("only floating joints can be transformed freely");
		}
		updateBulletTrafo(currentLocalTrafo, trafo);
	}

	void updateWorldTrafos() {
		auto worldToParent = parent ?
			parent->bulletObject.getWorldTransform() : btTransform::getIdentity();
		bulletObject.setWorldTransform(worldToParent * currentLocalTrafo);
		for (const auto& child : children) {
			child->updateWorldTrafos();
		}
	}

	double getCurrentJointValue() const {
		return currentJointValue;
	}

	btTransform getCurrentLocalTrafo() const {
		return currentLocalTrafo;
	}
};

struct ByJoint {
	const std::string& operator() (const PartDescriptor& pd) {
		return pd.jointId;
	}
	const std::string& operator() (const std::unique_ptr<Part>& p) {
		return p->jointId;
	}
};

struct ByLink {
	const std::string& operator() (const PartDescriptor& pd) {
		return pd.linkId;
	}
	const std::string& operator() (const std::unique_ptr<Part>& p) {
		return p->linkId;
	}
};

inline const ByJoint byJoint;
inline const ByLink byLink;

using MultiBodyDescriptor = DualRegistry<PartDescriptor, ByJoint, ByLink>;
using MultiBody = DualRegistry<unique_ptr<Part>, ByJoint, ByLink>;