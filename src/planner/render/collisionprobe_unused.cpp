
#include "LinearMath/btQuickprof.h"
#include "dualmc/dualmc.h"
#include "collisionprobe.h"
#include "collision/collisionchecker.h"
#include "html/renderer.html.h"

string probeCollisionHulls(
	btCollisionWorld* world,
	double resolution,
	btCollisionShape* probe
) {

#ifdef BT_ENABLE_PROFILE
	btSetCustomEnterProfileZoneFunc(CProfileManager::Start_Profile);
	btSetCustomLeaveProfileZoneFunc(CProfileManager::Stop_Profile);
#endif

	auto collisionConfiguration = make_unique<btDefaultCollisionConfiguration>();
	auto dispatcher = make_unique<btCollisionDispatcher>(collisionConfiguration.get());
	auto pairCache = make_unique<btHashedOverlappingPairCache>();
	auto broadphaseInterface = make_unique<btDbvtBroadphase>(pairCache.get());
	auto worldCopy = make_unique<btCollisionWorld>(
		dispatcher.get(),
		broadphaseInterface.get(),
		collisionConfiguration.get()
		);

	vector<unique_ptr<btCollisionObject>> objectCopies;
	for (size_t i = 0; i < world->getNumCollisionObjects(); i++) {
		auto obj = world->getCollisionObjectArray()[i];
		auto copy = make_unique<btCollisionObject>();
		copy->setCollisionShape(obj->getCollisionShape());
		copy->setWorldTransform(obj->getWorldTransform());
		worldCopy->addCollisionObject(copy.get(), OBJECT_GROUP, OBJECT_MASK);
		objectCopies.push_back(move(copy));
	}

	auto& objs = worldCopy->getCollisionObjectArray();

	btVector3 aabbMin(MAX, MAX, MAX);
	btVector3 aabbMax(MIN, MIN, MIN);
	for (size_t i = 0; i < objs.size(); i++) {
		btVector3 objAabbMin;
		btVector3 objAabbMax;
		objs[i]->getCollisionShape()->getAabb(
			objs[i]->getWorldTransform(), objAabbMin, objAabbMax);
		aabbMin.setValue(
			std::min(aabbMin[0], objAabbMin[0]),
			std::min(aabbMin[1], objAabbMin[1]),
			std::min(aabbMin[2], objAabbMin[2])
		);
		aabbMax.setValue(
			std::max(aabbMax[0], objAabbMax[0]),
			std::max(aabbMax[1], objAabbMax[1]),
			std::max(aabbMax[2], objAabbMax[2])
		);
	}
	aabbMin -= 1 * btVector3(resolution, resolution, resolution);
	aabbMax += 3 * btVector3(resolution, resolution, resolution);

	auto defaultProbe = make_unique<btSphereShape>(1e-6);
	if (probe == nullptr) {
		probe = defaultProbe.get();
	}

	size_t nx = std::ceil((aabbMax[0] - aabbMin[0]) / resolution);
	size_t ny = std::ceil((aabbMax[1] - aabbMin[1]) / resolution);
	size_t nz = std::ceil((aabbMax[2] - aabbMin[2]) / resolution);

	cout << "generating probing points..." << "\n";

	vector<unique_ptr<btCollisionObject>> points;
	points.reserve(nx * ny * nz);
	cout << nx * ny * nz << "\n";

	size_t iCell = 0;
	for (size_t iz = 0; iz < nz; iz++) {
		double z = aabbMin[2] + iz * resolution;
		for (size_t iy = 0; iy < ny; iy++) {
			double y = aabbMin[1] + iy * resolution;
			for (size_t ix = 0; ix < nx; ix++) {
				double x = aabbMin[0] + ix * resolution;

				auto point = make_unique<btCollisionObject>();
				point->getWorldTransform().getOrigin().setValue(x, y, z);
				point->setCollisionShape(probe);
				worldCopy->addCollisionObject(point.get(), PROBE_GROUP, PROBE_MASK);
				point->setUserIndex(PROBE_TYPE);
				point->setUserIndex2(iCell);

				points.push_back(move(point));
				iCell++;
			}
		}
	}

	cout << "running collision detection..." << "\n";

	worldCopy->performDiscreteCollisionDetection();

	cout << "reading collision data..." << "\n";

	vector<int8_t> data(nx * ny * nz, 0);

	for (int i = 0; i < worldCopy->getDispatcher()->getNumManifolds(); i++) {
		btPersistentManifold* contactManifold =
			worldCopy->getDispatcher()->getManifoldByIndexInternal(i);
		for (int j = 0; j < contactManifold->getNumContacts(); j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0) {
				auto objA = contactManifold->getBody0();
				auto objB = contactManifold->getBody1();
				if (objA->getUserIndex() == PROBE_TYPE) {
					data[objA->getUserIndex2()] = 10;
				}
				if (objB->getUserIndex() == PROBE_TYPE) {
					data[objB->getUserIndex2()] = 10;
				}
			}
		}
	}

	cout << "generating html output..." << "\n";

	dualmc::DualMC<int8_t> builder;
	vector<dualmc::Vertex> vertices;
	vector<dualmc::Quad> quads;

	builder.build(data.data(), nx, ny, nz, 5, false, false, vertices, quads);

	stringstream ss;

	objs = worldCopy->getCollisionObjectArray();
	for (size_t i = 0; i < objs.size(); i++) {
		auto trafo = objs[i]->getWorldTransform();
		auto shape = objs[i]->getCollisionShape();
		//ss << shapeToHtml(shape, trafo);
	}

	ss << "mesh([\n";
	for (const auto& v : vertices) {
		ss
			<< v.x * resolution + aabbMin[0] << ","
			<< v.y * resolution + aabbMin[1] << ","
			<< v.z * resolution + aabbMin[2] << ",\n";
	}
	ss << "],[\n";
	for (const auto& q : quads) {
		ss << q.i0 << "," << q.i1 << "," << q.i2 << ", "
			<< q.i0 << "," << q.i2 << "," << q.i3 << ",\n";
	}
	ss << "], false, meshMaterial)";

	cout << "cleaning up" << "\n";

#ifdef BT_ENABLE_PROFILE
	CProfileManager::dumpAll();
#endif

	// return str::replace(rendererHtml, "/*OBJECTS*/", ss.str());
	return ss.str();
}

