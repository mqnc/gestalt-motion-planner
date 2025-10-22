
#include "stl.h"
#include "test_main.h"
#include "btBulletCollisionCommon.h"

TEST(test_bullet, simple_collision) {
	auto collisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
	auto dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfiguration.get());
	auto overlappingPairCache = std::make_unique<btDbvtBroadphase>();
	auto collisionWorld = std::make_unique<btCollisionWorld>(
		dispatcher.get(), overlappingPairCache.get(), collisionConfiguration.get());

	auto colShape = std::make_unique<btSphereShape>(btScalar(1.));

	auto o1 = std::make_unique<btCollisionObject>();
	o1->setCollisionShape(colShape.get());
	collisionWorld->addCollisionObject(o1.get());

	auto o2 = std::make_unique<btCollisionObject>();
	o2->setCollisionShape(colShape.get());
	collisionWorld->addCollisionObject(o2.get());

	const double eps = 2 * std::numeric_limits<double>::epsilon();

	for (double x : {2.0 - eps, 2.0 + eps}) {
		o2->getWorldTransform().getOrigin()[0] = x;
		collisionWorld->performDiscreteCollisionDetection();

		int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
		int totalCandidates = 0;
		int totalCollisions = 0;

		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold* contactManifold =
				collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
			int numCandidates = contactManifold->getNumContacts();
			totalCandidates += numCandidates;
			for (int j = 0; j < numCandidates; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance() < 0) {
					totalCollisions++;
				}
			}
		}

		const int expectedCollisions = x < 2.0 ? 1 : 0;
		EXPECT_EQ(totalCollisions, expectedCollisions)
			<< "bullet must be built with double precision (BT_USE_DOUBLE_PRECISION)";
	}

	EXPECT_LT(
		btQuaternion(1, 2, 3).angleShortestPath(
			btQuaternion(btVector3(0, 0, 1), 1)
			* btQuaternion(btVector3(0, 1, 0), 2)
			* btQuaternion(btVector3(1, 0, 0), 3)
		), 1e-12
	) << "euler angles must match yaw pitch roll (compile with BT_EULER_DEFAULT_ZYX)";

}

TEST_MAIN
