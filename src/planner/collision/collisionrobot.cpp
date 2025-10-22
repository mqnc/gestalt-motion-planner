
#include "collisionrobot.h"

auto btTransformFromUrdf(const UrdfTrafo& ut)
{
	return btTransform(
		btQuaternion(ut.rpy[2], ut.rpy[1], ut.rpy[0]),
		btVector3(ut.xyz[0], ut.xyz[1], ut.xyz[2]));
}

PartDescriptor partDescriptorFromUrdf(
	const UrdfJoint& joint,
	const UrdfLink& link,
	const dict<string>& meshSources)
{

	btTransform origin = btTransform::getIdentity();
	btVector3 axis = {0, 0, 0};
	array<double, 2> limits = {NaN, NaN};

	const auto type = jointTypeFromString(joint.type);

	if (type == JointType::prismatic || type == JointType::revolute) {
		limits = joint.limit;
	}
	if (type == JointType::prismatic || type == JointType::revolute || type == JointType::continuous) {
		axis = btVector3(joint.axis[0], joint.axis[1], joint.axis[2]);
	}
	if (type == JointType::fixed || type == JointType::prismatic || type == JointType::revolute || type == JointType::continuous) {
		origin = btTransform(
			btQuaternion(joint.origin.rpy[2], joint.origin.rpy[1], joint.origin.rpy[0]),
			btVector3(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]));
	}

	auto linkCollisionShape = hold<CompoundShape>();
	auto linkVisualShape = hold<CompoundShape>();

	for (const auto& geom: link.collisionGeometries) {

		auto jointToGeom = btTransformFromUrdf(geom.origin);
		Holder<Shape> collisionChildShape;
		Holder<Shape> visualChildShape;

		if (geom.type == "box") {
			collisionChildShape = hold<BoxShape>(
				geom.size[0], geom.size[1], geom.size[2]);
			visualChildShape = hold<BoxShape>(
				geom.size[0], geom.size[1], geom.size[2]);
		}
		else if (geom.type == "sphere") {
			collisionChildShape = hold<SphereShape>(geom.radius);
			visualChildShape = hold<SphereShape>(geom.radius);
		}
		else if (geom.type == "cylinder") {
			collisionChildShape = hold<CylinderZShape>(
				geom.radius, geom.length);
			visualChildShape = hold<CylinderZShape>(
				geom.radius, geom.length);
		}
		else if (geom.type == "mesh") {

			string file = geom.filename;
			assert(meshSources.count(file));
			string source = meshSources.at(file);

			stl_reader::StlMesh<double, unsigned int> mesh(source);

			// extract vertices
			vector<btVector3> vertices;
			for (size_t ivrt = 0; ivrt < mesh.num_vrts(); ivrt++) {
				auto* v = mesh.vrt_coords(ivrt);
				vertices.push_back(btVector3(v[0], v[1], v[2]));
			}

			// extract faces
			vector<array<unsigned int, 3>> faces;
			for (size_t itri = 0; itri < mesh.num_tris(); itri++) {
				faces.push_back({
					mesh.tri_corner_ind(itri, 0),
					mesh.tri_corner_ind(itri, 1),
					mesh.tri_corner_ind(itri, 2)});
			}

			// determine if mesh is convex
			// convex if all mesh vrtices are on the same side of all faces
			// (side as in side of the 3d plane that the face is a part of)
			bool isConvex = true;
			for (const auto& face: faces) {
				auto v0 = vertices[face[0]];
				auto v1 = vertices[face[1]];
				auto v2 = vertices[face[2]];
				auto normal = (v1 - v0).cross(v2 - v0);
				bool someCornersOnPositiveFaceSide = false;
				bool someCornersOnNegativeFaceSide = false;
				for (const auto& meshCorner: vertices) {
					if (normal.dot(meshCorner - v0) > 1e-6) {
						someCornersOnPositiveFaceSide = true;
					}
					if (normal.dot(meshCorner - v0) < -1e-6) {
						someCornersOnNegativeFaceSide = true;
					}
					if (someCornersOnPositiveFaceSide && someCornersOnNegativeFaceSide) {
						isConvex = false;
						break;
					}
				}
				if (!isConvex) { break; }
			}

			// std::cout << geom.filename << " is " << (isConvex? "convex\n":"concave\n");

			// create concave shape for visualization (and collision if concave)
			auto triangles = make_shared<btTriangleMesh>();
			for (const auto& face: faces) {
				triangles->addTriangle(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
			}
			auto concaveMesh = hold<ConcaveTriangleMeshShape>(triangles);

			if (isConvex) {
				// create convex hull shape for collision
				auto convexHull = hold<ConvexHullShape>();
				for (const auto& v: vertices) {
					convexHull->addPoint(v.x(), v.y(), v.z());
				}
				convexHull->optimize();
				visualChildShape = move(concaveMesh);
				collisionChildShape = move(convexHull);
			}
			else {
				visualChildShape = concaveMesh;
				collisionChildShape = move(concaveMesh);
			}
		}
		else {
			throw runtime_error("collision shape "s + geom.type + " not supported");
		}

		if (collisionChildShape) {
			linkCollisionShape->addShape(jointToGeom, move(collisionChildShape));
		}
		if (visualChildShape) {
			linkVisualShape->addShape(jointToGeom, move(visualChildShape));
		}
	}

	return {
		joint.name,
		link.name,
		type,
		origin,
		axis,
		limits,
		joint.parent,
		move(linkCollisionShape),
		move(linkVisualShape)};
}

CollisionRobotTemplate::CollisionRobotTemplate(
	const string& id,
	const UrdfRobot& urdfRobot,
	const dict<string>& meshSources,
	const vector<string>& selectedJoints,
	const dict<vector<string>>& collisionIgnoreGroups,
	const bool encapsulate): id {id},
	collisionIgnoreGroups {collisionIgnoreGroups},
	selectedJoints {selectedJoints}
{
	dict<string> linkParents;

	for (const auto& joint: urdfRobot.joints) {
		auto& link = *urdfRobot.links.find(joint.child);
		auto [_, success] = linkParents.insert({link.name, joint.name});
		assert(success);
		partDescriptors.insert(partDescriptorFromUrdf(joint, link, meshSources));
	}

	// validate list of actuated joints
	for (const auto& joint: selectedJoints) {
		if (!partDescriptors.contains(byJoint, joint) || partDescriptors.find(byJoint, joint)->type == JointType::fixed || partDescriptors.find(byJoint, joint)->type == JointType::floating) {
			throw runtime_error(joint + " is not a joint that can be actuated");
		}
	}

	// find root link (the one that has no parent)
	string rootLink = "";
	for (const auto& link: urdfRobot.links) {
		if (linkParents.count(link.name) == 0) {
			if (rootLink == "") {
				rootLink = link.name;
			}
			else {
				throw runtime_error(urdfRobot.name + " has multiple root links");
			}
		}
	}
	if (rootLink == "") {
		throw runtime_error(urdfRobot.name + " has no root link");
	}

	partDescriptors.insert(partDescriptorFromUrdf(
		UrdfJoint {"__root__", "floating"},
		*urdfRobot.links.find(rootLink), meshSources));
}

CollisionRobot::CollisionRobot(
	const CollisionRobotTemplate& tmpl,
	const string& id): id {id}
{
	for (const auto& partDescriptor: tmpl.partDescriptors) {
		parts.insert(make_unique<Part>(partDescriptor, id));
	}

	preSelectedJoints = tmpl.selectedJoints;
	selectedJoints = tmpl.selectedJoints;
	collisionIgnoreGroups = tmpl.collisionIgnoreGroups;

	init();
}

void CollisionRobot::init()
{

	root = &getPartByJointId("__root__");

	// hook up relations
	for (auto& part: parts) {
		if (part.get() != root) {
			part->setParent(&getPartByLinkId(part->parentLinkId));
		}
	}
	for (const auto& j: selectedJoints) {
		jointMap.push_back(&getPartByJointId(j));
	}

	// personalize collision ignore groups (prefix with id)
	auto groupTemplates = collisionIgnoreGroups;
	collisionIgnoreGroups.clear();

	for (const auto& [groupId, members]: groupTemplates) {
		string myGroup = id + "." + groupId;
		collisionIgnoreGroups[myGroup] = {};
		for (const auto& link: members) {
			collisionIgnoreGroups[myGroup].push_back(id + "." + link);
		}
	}
}

const pair<CollisionRobot*, Part*> CollisionRobot::getParent() const{
	return {parent, parentPart};
}

void CollisionRobot::setParent(CollisionRobot* newParent, string newParentLinkId)
{
	parentPart = nullptr;
	// won't get a new parent when removing robot from scene
	if (newParent != nullptr) {
		parentPart = newParentLinkId == "__root__" ? &(newParent->getPartByJointId("__root__"))
														: &(newParent->getPartByLinkId(newParentLinkId));
	}
	// doesn't have a parent when parent is assigned during spawning
	if (parent) {
		parent->children.erase(this);
	}
	if (newParent != nullptr) {
		parent = newParent;
		parent->children.insert(this);
	}
	root->setParent(parentPart);
}

const btTransform& CollisionRobot::getParentTrafo()
{
	if (parentPart) {
		return parentPart->bulletObject.getWorldTransform();
	}
	else {
		return btTransform::getIdentity();
	}
}

void CollisionRobot::setBaseTrafo(const btTransform& trafo)
{
	root->setJointTrafo(trafo);
	root->updateWorldTrafos();
}

btTransform CollisionRobot::getPartTrafoInWorld(string linkId)
{
	if (linkId == "__root__") {
		return root->bulletObject.getWorldTransform();
	}
	else {
		return getPartByLinkId(linkId).bulletObject.getWorldTransform();
	}
}

void CollisionRobot::selectJoints(const vector<string>& ids)
{
	if (ids.size() == 0) {
		selectedJoints = preSelectedJoints;
	}
	else {
		// validate list of actuated joints
		for (const auto& j: ids) {
			if (!parts.contains(byJoint, j)) {
				throw runtime_error(id + " has no joint named " + j);
			}
			else if (parts.at(byJoint, j)->type == JointType::fixed) {
				throw runtime_error(id + "." + j + " is a fixed joint");
			}
			else if (parts.at(byJoint, j)->type == JointType::floating) {
				throw runtime_error( id + "." + j + " is a free joint (only used internally for positioning object roots)");
			}
		}
		selectedJoints = ids;
	}
	jointMap.clear();
	for (const auto& j: selectedJoints) {
		jointMap.push_back(&getPartByJointId(j));
	}
}

const vector<string>& CollisionRobot::getJointSelection() const
{
	return selectedJoints;
}

valarray<double> CollisionRobot::getJointPositions() const
{
	valarray<double> result(jointMap.size());
	for (size_t j = 0; j < jointMap.size(); j++) {
		result[j] = jointMap[j]->getCurrentJointValue();
	}
	return result;
}

void CollisionRobot::setJointPositions(const valarray<double>& jointValues)
{
	if (jointMap.size() == 0 && jointValues.size() != 0) {
		throw runtime_error(string("") + "no joints selected; load the robot (" + id + ") with a configuration file or use select_joints function");
	}
	else if (jointMap.size() != jointValues.size()) {
		throw runtime_error("wrong number of joint positions specified");
	}

	for (size_t j = 0; j < jointValues.size(); j++) {
		if (!std::isnan(jointValues[j])) {
			jointMap[j]->setJointValue(jointValues[j]);
		}
	}
	root->updateWorldTrafos();
}

bool CollisionRobot::checkJointLimits(const valarray<double>& jointValues) const
{
	for (size_t j = 0; j < jointValues.size(); j++) {
		if (
			jointValues[j] < getPartByJointIndex(j).limits[0] || jointValues[j] > getPartByJointIndex(j).limits[1]) {

			return false;
		}
	}
	return true;
}

bool CollisionRobot::checkJointLimits(const valarray<valarray<double>>& jointTrajectory) const
{
	for (size_t i = 0; i < jointTrajectory.size(); i++) {
		if (!checkJointLimits(jointTrajectory[i])) {
			return false;
		}
	}
	return true;
}

void CollisionRobot::getJointLimits(valarray<double>& upperLimits, valarray<double>& lowerLimits) const
{
	upperLimits.resize(jointMap.size());
	lowerLimits.resize(jointMap.size());

	for (size_t j = 0; j < jointMap.size(); j++) {
		upperLimits[j] = getPartByJointIndex(j).limits[1];
		lowerLimits[j] = getPartByJointIndex(j).limits[0];
	}
}

valarray<double> CollisionRobot::getMaxJointStepSizes(valarray<double> override) const
{
	valarray<double> result(jointMap.size());
	for (size_t j = 0; j < jointMap.size(); j++) {
		if (override.size() > j && !std::isnan(override[j])) {
			result[j] = override[j];
		}
		else {
			const auto& type = getPartByJointIndex(j).type;
			if (type == JointType::revolute || type == JointType::continuous) {
				result[j] = 1.0 * M_PI / 180.0; // 1 deg by default
			}
			if (type == JointType::prismatic) {
				result[j] = 0.01; // 1 cm by default
			}
		}
	}
	return result;
}

void CollisionRobot::setActuationState(bool actuated)
{
	isActuated = actuated;
	for (auto& child: children) {
		child->setActuationState(actuated);
	}
}

std::unordered_set<CollisionRobot*>& CollisionRobot::getChildren()
{
	return children;
}

Part& CollisionRobot::getPartByJointId(string id)
{
	return const_cast<Part&>(
		const_cast<const CollisionRobot*>(this)->getPartByJointId(id));
}

const Part& CollisionRobot::getPartByJointId(string id) const
{
	if (!parts.contains(byJoint, id)) {
		cout << "attempted to access joint \""
			 << id << "\" which does not exist in robot \""
			 << this->id << "\"; available joints: ";
		for (const auto& part: parts) {
			cout << '"' << part->jointId << "\" ";
		}
		cout << "\n";
		// still trigger standard exception below
	}
	return *parts.at(byJoint, id);
}

Part& CollisionRobot::getPartByJointIndex(size_t index)
{
	return const_cast<Part&>(
		const_cast<const CollisionRobot*>(this)->getPartByJointIndex(index));
}

const Part& CollisionRobot::getPartByJointIndex(size_t index) const
{
	if (index >= jointMap.size()) {
		cout << "attempted to access joint["
			 << index << "] but robot \""
			 << id << "\" only has " << jointMap.size()
			 << " selected actuated joints.";
		throw runtime_error("array limit exceeded");
	}
	return *(jointMap[index]);
}

Part& CollisionRobot::getPartByLinkId(string id)
{
	if (!parts.contains(byLink, id)) {
		cout << "attempted to access link \""
			 << id << "\" which does not exist in robot \""
			 << this->id << "\"; available links: ";
		for (const auto& part: parts) {
			cout << '"' << part->linkId << "\" ";
		}
		cout << "\n";
		// still trigger standard exception below
	}
	return *parts.at(byLink, id);
}

const MultiBody& CollisionRobot::getParts() const
{
	return parts;
}
MultiBody& CollisionRobot::getMutableParts()
{
	return parts;
}

void CollisionRobot::setMargin(double margin)
{
	for (const auto& part: parts) {
		part->collisionShape->setSafetyMargin(margin);
	}
}

void CollisionRobot::setPartMargin(string id, double margin)
{
	getPartByLinkId(id).collisionShape->setSafetyMargin(margin);
}

const dict<vector<string>>&
CollisionRobot::getCollisionIgnoreGroups() const
{
	return collisionIgnoreGroups;
}
