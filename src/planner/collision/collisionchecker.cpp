#include "collisionchecker.h"

bool CollisionChecker::locked = false;
const int GIMPACT_REGISTERED = 8000;

#ifndef BT_DEBUG_MEMORY_ALLOCATIONS
CollisionChecker::customCollisionConfiguration CollisionChecker::collisionConfiguration{};
btCollisionDispatcher CollisionChecker::dispatcher(&collisionConfiguration);
#endif

CollisionChecker::CollisionChecker(
	const vector<pair<btCollisionObject*, BitMask>>& objects
) :
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	collisionConfiguration{ std::make_unique<CollisionChecker::customCollisionConfiguration>() },
	dispatcher{ std::make_unique<btCollisionDispatcher>(collisionConfiguration.get()) },
#endif
	pairCache{ make_unique<btHashedOverlappingPairCache>() },
	filterCallback{ make_unique<FilterCallback>() },
	broadphaseInterface{ make_unique<btDbvtBroadphase>(pairCache.get()) },
#ifndef BT_DEBUG_MEMORY_ALLOCATIONS
	collisionWorld{ make_unique<btCollisionWorld>(
		&dispatcher,
		broadphaseInterface.get(),
		&collisionConfiguration
	) }
#else
	collisionWorld{ make_unique<btCollisionWorld>(
		dispatcher.get(),
		broadphaseInterface.get(),
		collisionConfiguration.get()
	) }
#endif
{
	if(!(dispatcher.getDispatcherFlags() & GIMPACT_REGISTERED)){
		btGImpactCollisionAlgorithm::registerAlgorithm(&dispatcher);
		dispatcher.setDispatcherFlags(dispatcher.getDispatcherFlags() | GIMPACT_REGISTERED);
	}

	if(locked){
		throw runtime_error("trying to run multiple collision checkers in parallel");
	}

	locked = true;

	pairCache->setOverlapFilterCallback(filterCallback.get());

	PROFILE_CTOR_STOP(CollisionChecker_construction);

	PROFILE_SCOPE(CollisionChecker_adding_objects);

	for(const auto &obj:objects){
		collisionWorld->addCollisionObject(obj.first, obj.second);
	}
}

CollisionChecker::~CollisionChecker() {

	PROFILE_SCOPE(CollisionChecker_removing_objects);

	auto objects = collisionWorld->getCollisionObjectArray();
	for (size_t i=0; i<objects.size(); i++) {
		collisionWorld->removeCollisionObject(objects[i]);
	}

	locked = false;

	PROFILE_DTOR_START(CollisionChecker_destruction);
}

CollisionChecker::CollisionReport CollisionChecker::checkCollisions(bool returnCollisions) {
	PROFILE_SCOPE(checkCollisions);
	collisionWorld->performDiscreteCollisionDetection();

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	int totalCandidates = 0;

	CollisionChecker::CollisionReport report;
	if (returnCollisions) {
		report.collisions = vector<Collision>();
	}

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold =
			collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		int numCandidates = contactManifold->getNumContacts();
		totalCandidates += numCandidates;
		for (int j = 0; j < numCandidates; j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0) {
				if (returnCollisions) {
					report.collisions->emplace_back(Collision{
						contactManifold->getBody0(),
						contactManifold->getBody1(),
						pt.getPositionWorldOnA(),
						pt.getPositionWorldOnB()
						});
				}
				report.numCollisions++;
			}
		}
	}

	return report;
}
