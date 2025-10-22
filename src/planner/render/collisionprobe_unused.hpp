
#pragma once

#include "common.h"
#include "utils.h"
#include "dualmc/dualmc.h"
#include "collision/collisionrobot.h"
#include "collision/collisionchecker.h"

// robot.bulletObject is temporarily mutated
inline Mesh probeRobot(CollisionRobot& robot) {
	const int res = 10;

	BitMask objectGroup = 0x1;
	BitMask probeGroup = 0x2;

	// we will let probe objects point to this and identify them by that
	int probeIdentifier = 0;

	vector<pair<btCollisionObject*, BitMask>> objects;

	for (auto& part : robot.getMutableParts()) {
		objects.push_back({
			&(part->bulletObject),
			objectGroup
			});
	}

	const double inf = std::numeric_limits<double>::infinity();

	btVector3 aabbMin(inf, inf, inf);
	btVector3 aabbMax(-inf, -inf, -inf);

	for (const auto& obj : objects) {
		btVector3 objAabbMin;
		btVector3 objAabbMax;
		obj.first->getCollisionShape()->getAabb(
			obj.first->getWorldTransform(), objAabbMin, objAabbMax);
		aabbMin.setMin(objAabbMin);
		aabbMax.setMax(objAabbMax);
	}
	//aabbMin -= 1 * btVector3(res, res, res);
	//aabbMax += 3 * btVector3(res, res, res);

	auto probe = make_unique<btSphereShape>(1e-6);

	size_t nx = res, ny = res, nz = res;
	btVector3 size = aabbMax - aabbMin;
	double dx = size.x() / (nx - 1);
	double dy = size.y() / (ny - 1);
	double dz = size.z() / (nz - 1);

	cout << "generating probing points..." << "\n";

	vector<unique_ptr<btCollisionObject>> points;
	points.reserve(nx * ny * nz);

	size_t iCell = 0;
	for (size_t iz = 0; iz < nz; iz++) {
		double z = aabbMin[2] + (-0.5 + iz) * dz;
		for (size_t iy = 0; iy < ny; iy++) {
			double y = aabbMin[1] + (-0.5 + iy) * dy;
			for (size_t ix = 0; ix < nx; ix++) {
				double x = aabbMin[0] + (-0.5 + ix) * dx;

				auto point = make_unique<btCollisionObject>();
				point->getWorldTransform().getOrigin().setValue(x, y, z);
				point->setCollisionShape(probe.get());
				point->setUserPointer(&probeIdentifier);
				point->setUserIndex(iCell);

				points.push_back(move(point));
				objects.push_back({ points.back().get(), probeGroup });
				iCell++;
			}
		}
	}

	CollisionChecker checker(objects);

	cout << "running collision detection..." << "\n";

	auto reports = checker.checkCollisions(true);

	cout << "reading collision data..." << "\n";

	vector<int8_t> data(nx * ny * nz, 0);
	for (const auto& col : *reports.collisions) {
		if (col.objectA->getUserPointer() == &probeIdentifier) {}
	}

	/*probeIdentifier
		cout << "generating html output..." << "\n";

		dualmc::DualMC<int8_t> builder;
		vector<dualmc::Vertex> vertices;
		vector<dualmc::Quad> quads;

		builder.build(data.data(), nx, ny, nz, 5, false, false, vertices, quads);

		stringstream ss;

		objects = worldCopy->getCollisionObjectArray();
		for (size_t i = 0; i < objects.size(); i++) {
			auto trafo = objects[i]->getWorldTransform();
			auto shape = objects[i]->getCollisionShape();
			//ss << shapeToHtml(shape, trafo);
		}

		ss << "mesh([\n";
		for (const auto& v : vertices) {
			ss
				<< v.x * res + aabbMin[0] << ","
				<< v.y * res + aabbMin[1] << ","
				<< v.z * res + aabbMin[2] << ",\n";
		}
		ss << "],[\n";
		for (const auto& q : quads) {
			ss << q.i0 << "," << q.i1 << "," << q.i2 << ", "
				<< q.i0 << "," << q.i2 << "," << q.i3 << ",\n";
		}
		ss << "], false, meshMaterial)";

		cout << "cleaning up" << "\n";
		*/
	return {};
}

