#pragma once

#include "common.h"
#include "profiling.h"

#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

class CollisionChecker {
	PROFILE_CTOR_FIRST_MEMBER(CollisionChecker_construction);

	// collision objects can only be part of one collision world at a time
	static bool locked;

	struct FilterCallback :public btOverlapFilterCallback {
		virtual bool needBroadphaseCollision(
			btBroadphaseProxy* proxy0,
			btBroadphaseProxy* proxy1
		) const {
			// if the two objects are in at least one common collision ignore group
			// (indicated by the bit mask), they can't collide
			return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterGroup) == 0;
		}
	};

	class customCollisionConfiguration :
		public btDefaultCollisionConfiguration {
	public:
#ifdef COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
		customCollisionConfiguration() {
			// remove bullets box box algorithm (which doesn't consider rounded edges for inflated objects)
			m_boxBoxCF->~btCollisionAlgorithmCreateFunc();
			btAlignedFree(m_boxBoxCF);

			// and replace it with bullets convex convex algorithm, which will round inflated edges
			void* mem = NULL;
			mem = btAlignedAlloc(sizeof(btConvexConvexAlgorithm::CreateFunc), 16);
			m_boxBoxCF = new (mem) btConvexConvexAlgorithm::CreateFunc(m_pdSolver);
		}
#endif
	};

	// profiling showed that these two eat a lot of time when initialized again for each scene
#ifndef BT_DEBUG_MEMORY_ALLOCATIONS
	static customCollisionConfiguration collisionConfiguration;
	static btCollisionDispatcher dispatcher;
#else
	const unique_ptr<customCollisionConfiguration> collisionConfiguration;
	const unique_ptr<btCollisionDispatcher> dispatcher;
#endif
	const unique_ptr<btOverlappingPairCache> pairCache;
	const unique_ptr<FilterCallback> filterCallback;
	const unique_ptr<btDbvtBroadphase> broadphaseInterface;
	const unique_ptr<btCollisionWorld> collisionWorld;

	PROFILE_DTOR_LAST_MEMBER(CollisionChecker_destruction);

public:
	CollisionChecker(
		const vector<pair<btCollisionObject*, BitMask>>& objects
	);

	~CollisionChecker();

	struct Collision {
		const btCollisionObject* objectA = nullptr;
		const btCollisionObject* objectB = nullptr;
		btVector3 pointOnA;
		btVector3 pointOnB;
	};

	struct CollisionReport {
		size_t numCollisions = 0;
		optional<vector<Collision>> collisions;
	};

	CollisionReport checkCollisions(
		bool returnCollisions = false
	);
};
