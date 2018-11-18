#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <glm/gtx/string_cast.hpp>
#include "logger/ILogger.hpp"

#include "Bullet.hpp"
#include "utilities/Utilities.hpp"
#include "physics/CollisionShapeHandle.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

namespace
{
struct QueryContactResultCallback : public btCollisionWorld::ContactResultCallback
{
	QueryContactResultCallback (logger::ILogger* logger, std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>>& result)
	:
		logger_(logger),
		result_(result)
	{
	}

    btScalar addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0Wrap,
        int partId0,
        int index0,
        const btCollisionObjectWrapper* colObj1Wrap,
        int partId1,
        int index1)
    {
    	if (cp.getDistance() < 0.0f)
    	{
			auto bulletUserData = static_cast<BulletUserData*>(colObj1Wrap->getCollisionObject()->getUserPointer());
			switch (bulletUserData->type)
			{
				case BulletCollisionObjectType::RIGID_BODY:
					result_.emplace_back(bulletUserData->rigidBodyObjectHandle);
					break;

				case BulletCollisionObjectType::GHOST:
					result_.emplace_back(bulletUserData->ghostObjectHandle);
					break;

				case BulletCollisionObjectType::UNKNOWN:
					LOG_WARN(logger_, "BulletCollisionObjectType set to Unknown");
					break;

				default:
					LOG_WARN(logger_, "Unknown BulletCollisionObjectType");
					break;
			}
    	}
    }

private:
    logger::ILogger* logger_;
    std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>>& result_;
};
}

Bullet::Bullet(ice_engine::utilities::Properties* properties, fs::IFileSystem* fileSystem, logger::ILogger* logger)
{
	properties_ = properties;
	fileSystem_ = fileSystem;
	logger_ = logger;
}

void Bullet::tick(const PhysicsSceneHandle& physicsSceneHandle, const float32 delta)
{
	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	physicsScene.dynamicsWorld->stepSimulation(delta, 1, 1.0f / 60.0f);

	if (physicsScene.debugRendering)
	{
		physicsScene.dynamicsWorld->debugDrawWorld();
	}
}

PhysicsSceneHandle Bullet::createPhysicsScene()
{
	auto physicsSceneHandle = physicsScenes_.create();
	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	physicsScene.broadphase = std::make_unique<btDbvtBroadphase>();
	physicsScene.collisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
	physicsScene.dispatcher = std::make_unique<btCollisionDispatcher>(physicsScene.collisionConfiguration.get());

	physicsScene.broadphase->resetPool(physicsScene.dispatcher.get());

	physicsScene.solver = std::make_unique<btSequentialImpulseConstraintSolver>();
	physicsScene.solver->reset();
	physicsScene.solver->setRandSeed(100);

	physicsScene.dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(physicsScene.dispatcher.get(), physicsScene.broadphase.get(), physicsScene.solver.get(), physicsScene.collisionConfiguration.get());

	physicsScene.dynamicsWorld->setGravity(btVector3(0, -10, 0));
	physicsScene.dynamicsWorld->getSolverInfo().m_solverMode = 0;
	physicsScene.dynamicsWorld->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	physicsScene.dynamicsWorld->setDebugDrawer(debugRenderer_.get());

	return physicsSceneHandle;
}

void Bullet::destroyPhysicsScene(const PhysicsSceneHandle& physicsSceneHandle)
{
	physicsScenes_.destroy(physicsSceneHandle);
}

void Bullet::setGravity(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& gravity)
{
	physicsScenes_[physicsSceneHandle].dynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}

void Bullet::setPhysicsDebugRenderer(IPhysicsDebugRenderer* physicsDebugRenderer)
{
	debugRenderer_ = std::make_unique<DebugRenderer>(physicsDebugRenderer);

	for (auto& scene : physicsScenes_)
	{
		scene.dynamicsWorld->setDebugDrawer(debugRenderer_.get());
	}
}

void Bullet::setDebugRendering(const PhysicsSceneHandle& physicsSceneHandle, const bool enabled)
{
	auto& physicsScene = physicsScenes_[physicsSceneHandle];
	physicsScene.debugRendering = enabled;
}

CollisionShapeHandle Bullet::createStaticPlaneShape(const glm::vec3& planeNormal, const float32 planeConstant)
{
	auto shape = std::make_unique<btStaticPlaneShape>(btVector3(planeNormal.x, planeNormal.y, planeNormal.z), btScalar(planeConstant));

	shapes_.push_back(std::move(shape));
	auto index = shapes_.size() - 1;

	return CollisionShapeHandle(index);
}

CollisionShapeHandle Bullet::createStaticBoxShape(const glm::vec3& dimensions)
{
	auto shape = std::make_unique<btBoxShape>(btVector3(dimensions.x, dimensions.y, dimensions.z));

	shapes_.push_back(std::move(shape));
	auto index = shapes_.size() - 1;

	return CollisionShapeHandle(index, 1);
}

std::vector<char> heightMapData;
CollisionShapeHandle Bullet::createStaticTerrainShape(const IHeightfield* heightfield)
{
	//(128, 128, data, 1, -1024, 1016, 2, PHY_UCHAR, true);
	heightMapData = heightfield->data();

	auto shape = std::make_unique<btHeightfieldTerrainShape>(heightfield->width(), heightfield->length(), &heightMapData[0], 1.0f/255.0f, -1.0f, 1.0f, 1, PHY_UCHAR, true);

	float32 scale = 1.0f;
	btVector3 localScaling = btVector3(scale, 15.0f, scale);
	shape->setLocalScaling(localScaling);

	shapes_.push_back(std::move(shape));
	auto index = shapes_.size() - 1;

	return CollisionShapeHandle(index, 1);
}

void Bullet::destroyStaticShape(const CollisionShapeHandle& collisionShapeHandle)
{
	throw std::logic_error("Method not implemented");
}

void Bullet::destroyAllStaticShapes()
{
	shapes_.clear();
}

RigidBodyObjectHandle Bullet::createRigidBodyObject(
	const PhysicsSceneHandle& physicsSceneHandle,
	const CollisionShapeHandle& collisionShapeHandle,
	std::unique_ptr<IMotionChangeListener> motionStateListener,
	const boost::any& userData
)
{
	return createRigidBodyObject(physicsSceneHandle, collisionShapeHandle, glm::vec3(), glm::quat(), 1.0f, 1.0f, 1.0f, std::move(motionStateListener), userData);
}

RigidBodyObjectHandle Bullet::createRigidBodyObject(
	const PhysicsSceneHandle& physicsSceneHandle,
	const CollisionShapeHandle& collisionShapeHandle,
	const float32 mass,
	const float32 friction,
	const float32 restitution,
	std::unique_ptr<IMotionChangeListener> motionStateListener,
	const boost::any& userData
)
{
	return createRigidBodyObject(physicsSceneHandle, collisionShapeHandle, glm::vec3(), glm::quat(), mass, friction, restitution, std::move(motionStateListener), userData);
}

RigidBodyObjectHandle Bullet::createRigidBodyObject(
	const PhysicsSceneHandle& physicsSceneHandle,
	const CollisionShapeHandle& collisionShapeHandle,
	const glm::vec3& position,
	const glm::quat& orientation,
	const float32 mass,
	const float32 friction,
	const float32 restitution,
	std::unique_ptr<IMotionChangeListener> motionStateListener,
	const boost::any& userData
)
{
	auto& shape = shapes_[collisionShapeHandle.index()];

	btTransform transform;
	transform.setRotation( btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w) );
	transform.setOrigin( btVector3(position.x, position.y, position.z) );

	btScalar bulletMass(mass);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	if (bulletMass != btScalar(0.0f))
	{
		shape->calculateLocalInertia(bulletMass, localInertia);
	}
	
	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	auto rigidBodyObjectHandle = physicsScene.rigidBodyData.create();//std::make_unique<BulletRigidBodyData>();
	auto& rigidBodyData = physicsScene.rigidBodyData[rigidBodyObjectHandle];

	rigidBodyData.motionState = std::make_unique<BulletMotionState>(transform, std::move(motionStateListener));
	btRigidBody::btRigidBodyConstructionInfo constructionInfo(bulletMass, rigidBodyData.motionState.get(), shape.get(), localInertia);
	constructionInfo.m_friction = friction;
	constructionInfo.m_restitution = restitution;
	rigidBodyData.rigidBody = std::make_unique<btRigidBody>(constructionInfo);

	rigidBodyData.userData = std::make_unique<BulletUserData>();

	rigidBodyData.userData->rigidBodyObjectHandle = rigidBodyObjectHandle;
	rigidBodyData.userData->type = BulletCollisionObjectType::RIGID_BODY;
	rigidBodyData.userData->userData = userData;
	rigidBodyData.rigidBody->setUserPointer(rigidBodyData.userData.get());

	//auto rigidBodyDataPointer = &rigidBodyData;
	auto rigidBodyPointer = rigidBodyData.rigidBody.get();
	//rigidBodyPointer->setUserPointer(rigidBodyDataPointer);

	physicsScene.dynamicsWorld->addRigidBody(rigidBodyPointer);

	return rigidBodyObjectHandle;
}
/*
RigidBodyObjectHandle Bullet::createRigidBodyObject(
	const PhysicsSceneHandle& physicsSceneHandle,
	const CollisionShapeHandle& collisionShapeHandle,
	const glm::vec3& position,
	const glm::quat& orientation,
	const float32 friction,
	const float32 restitution,
	const boost::any& userData
)
{
	auto& shape = shapes_[collisionShapeHandle.index()];

	btTransform transform;
	transform.setRotation( btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w) );
	transform.setOrigin( btVector3(position.x, position.y, position.z) );

	btVector3 localInertia(0.0f, 0.0f, 0.0f);

	auto rigidBodyData = std::make_unique<BulletRigidBodyData>();

	rigidBodyData->motionState = std::make_unique<BulletMotionState>(transform, nullptr);
	btRigidBody::btRigidBodyConstructionInfo constructionInfo(btScalar(0.0f), rigidBodyData->motionState.get(), shape.get(), localInertia);
	constructionInfo.m_friction = friction;
	constructionInfo.m_restitution = restitution;
	rigidBodyData->rigidBody = std::make_unique<btRigidBody>(constructionInfo);

	rigidBodyData->userData = std::make_unique<BulletUserData>();

	rigidBodyData->userData->rigidBodyObjectHandle = RigidBodyObjectHandle(rigidBodyData.get());
	rigidBodyData->userData->type = BulletCollisionObjectType::RIGID_BODY;
	rigidBodyData->userData->userData = userData;

	auto rigidBodyDataPointer = rigidBodyData.get();
	auto rigidBodyPointer = rigidBodyData->rigidBody.get();

	rigidBodyData_.push_back(std::move(rigidBodyData));

	rigidBodyPointer->setUserPointer(rigidBodyDataPointer);

	physicsScenes_[physicsSceneHandle].dynamicsWorld->addRigidBody(rigidBodyPointer);

	return RigidBodyObjectHandle(rigidBodyDataPointer);
}
*/
GhostObjectHandle Bullet::createGhostObject(const PhysicsSceneHandle& physicsSceneHandle, const CollisionShapeHandle& collisionShapeHandle, const boost::any& userData)
{
	return createGhostObject(physicsSceneHandle, collisionShapeHandle, glm::vec3(), glm::quat(), userData);
}

GhostObjectHandle Bullet::createGhostObject(
	const PhysicsSceneHandle& physicsSceneHandle,
	const CollisionShapeHandle& collisionShapeHandle,
	const glm::vec3& position,
	const glm::quat& orientation,
	const boost::any& userData
)
{
	auto& shape = shapes_[collisionShapeHandle.index()];

	btTransform transform;
	transform.setRotation( btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w) );
	transform.setOrigin( btVector3(position.x, position.y, position.z) );

	//btVector3 localInertia(0.0f, 0.0f, 0.0f);
	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	auto ghostObjectHandle = physicsScene.ghostObjectData.create();//std::make_unique<BulletGhostObjectData>();
	auto& ghostObjectData = physicsScene.ghostObjectData[ghostObjectHandle];

	//ghostObjectData->motionState = std::make_unique<BulletMotionState>(transform, nullptr);
	//btRigidBody::btRigidBodyConstructionInfo constructionInfo(btScalar(0.0f), ghostObjectData->motionState.get(), shape.get(), localInertia);
	//constructionInfo.m_friction = friction;
	//constructionInfo.m_restitution = restitution;
	ghostObjectData.ghostObject = std::make_unique<btGhostObject>();
	ghostObjectData.ghostObject->setCollisionShape(shape.get());
	ghostObjectData.ghostObject->setWorldTransform(transform);
	ghostObjectData.ghostObject->setCollisionFlags(ghostObjectData.ghostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

	ghostObjectData.userData = std::make_unique<BulletUserData>();

	ghostObjectData.userData->ghostObjectHandle = ghostObjectHandle;
	ghostObjectData.userData->type = BulletCollisionObjectType::GHOST;
	ghostObjectData.userData->userData = userData;
	ghostObjectData.ghostObject->setUserPointer(ghostObjectData.userData.get());

	//auto ghostDataPointer = &ghostObjectData;
	auto ghostPointer = ghostObjectData.ghostObject.get();
	//ghostPointer->setUserPointer(ghostDataPointer);

	physicsScene.dynamicsWorld->addCollisionObject(ghostPointer, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter);

	//physicsScenes_[physicsSceneHandle].dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	return ghostObjectHandle;
}

void Bullet::destroy(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle)
{
	//rigidBodyData_.destroy(rigidBodyObjectHandle);

	BulletRigidBodyData* rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	auto rigidBodyPointer = rigidBodyData->rigidBody.get();

	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	physicsScene.dynamicsWorld->removeRigidBody(rigidBodyPointer);
	physicsScene.rigidBodyData.destroy(rigidBodyObjectHandle);
}

void Bullet::destroy(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle)
{
	//rigidBodyData_.destroy(rigidBodyObjectHandle);

	BulletGhostObjectData* ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	auto ghostPointer = ghostObjectData->ghostObject.get();

	auto& physicsScene = physicsScenes_[physicsSceneHandle];

	physicsScene.dynamicsWorld->removeCollisionObject(ghostPointer);
	physicsScene.ghostObjectData.destroy(ghostObjectHandle);
}

void Bullet::destroyAllRigidBodies()
{
	//rigidBodyData_.clear();
}

void Bullet::setUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const boost::any& userData)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	//rigidBodyData->userData = std::make_unique<BulletUserData>();

	//rigidBodyData->userData->rigidBodyObjectHandle = rigidBodyObjectHandle;
	//rigidBodyData->userData->type = BulletCollisionObjectType::RIGID_BODY;
	rigidBodyData->userData->userData = userData;
	//rigidBodyData->rigidBody->setUserPointer(rigidBodyData->userData.get());
}

void Bullet::setUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const boost::any& userData)
{
	auto ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	//ghostObjectData->userData = std::make_unique<BulletUserData>();

	//ghostObjectData->userData->ghostObjectHandle = ghostObjectHandle;
	//ghostObjectData->userData->type = BulletCollisionObjectType::GHOST;
	ghostObjectData->userData->userData = userData;
	//ghostObjectData->ghostObject->setUserPointer(ghostObjectData->userData.get());
}

boost::any& Bullet::getUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	return rigidBodyData->userData->userData;
}

boost::any& Bullet::getUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const
{
	auto ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	return ghostObjectData->userData->userData;
}

Raycast Bullet::raycast(const PhysicsSceneHandle& physicsSceneHandle, const ray::Ray& ray)
{
	Raycast result;
	result.setRay(ray);

	btVector3 bulletVectorStart = utilities::convert(ray.from);
	btVector3 bulletVectorEnd = utilities::convert(ray.to);

	btCollisionWorld::ClosestRayResultCallback rayCallback(bulletVectorStart, bulletVectorEnd);

	// Perform raycast
	physicsScenes_[physicsSceneHandle].dynamicsWorld->rayTest(bulletVectorStart, bulletVectorEnd, rayCallback);

	if (rayCallback.hasHit())
	{
	    result.setHitPointWorld(utilities::convert(rayCallback.m_hitPointWorld));
	    result.setHitNormalWorld(utilities::convert(rayCallback.m_hitNormalWorld));

	    if (rayCallback.m_collisionObject != nullptr)
	    {
			if (rayCallback.m_collisionObject->getUserPointer() == nullptr)
			{
				LOG_WARN(logger_, "Bullet collision object has a null user pointer.");
			}

			auto bulletUserData = static_cast<BulletUserData*>(rayCallback.m_collisionObject->getUserPointer());
			switch (bulletUserData->type)
			{
				case BulletCollisionObjectType::RIGID_BODY:
					result.setRigidBodyObjectHandle(bulletUserData->rigidBodyObjectHandle);
					break;

				case BulletCollisionObjectType::GHOST:
					result.setGhostObjectHandle(bulletUserData->ghostObjectHandle);
					break;

				case BulletCollisionObjectType::UNKNOWN:
					LOG_WARN(logger_, "BulletCollisionObjectType set to Unknown");
					break;

				default:
					LOG_WARN(logger_, "Unknown BulletCollisionObjectType");
					break;
			}
		}
	}

	return result;
}

std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> Bullet::query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const std::vector<glm::vec3>& points)
{
	std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> result;

//	btConvexHullShape*  shape = new btConvexHullShape(reinterpret_cast<const btScalar*>(&points[0]), points.size(), sizeof(glm::vec3));
	btConvexHullShape shape;
	for (const auto& point : points) shape.addPoint(utilities::convert(point));

//	btSphereShape*  shape = new btSphereShape(4);
	btPairCachingGhostObject ghost;

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(utilities::convert(origin));

//	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
//	btRigidBody::btRigidBodyConstructionInfo constructionInfo(1, myMotionState, shape);
//	btRigidBody*  ghost = new btRigidBody(constructionInfo);

	ghost.setCollisionShape(&shape);
	ghost.setWorldTransform(transform);
	ghost.setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

	auto& dynamicsWorld = physicsScenes_[physicsSceneHandle].dynamicsWorld;
	dynamicsWorld->addCollisionObject(&ghost);

	QueryContactResultCallback  resultCallback(logger_, result);

	for (int i = 0; i < ghost.getNumOverlappingObjects(); ++i)
	{
	    auto object = ghost.getOverlappingObject(i);

	    if (object != nullptr)
		{
			if (object->getUserPointer() == nullptr)
			{
				LOG_WARN(logger_, "Bullet collision object has a null user pointer.");
			}

			dynamicsWorld->contactPairTest(&ghost, object, resultCallback);
		}
	}

//	dynamicsWorld->performDiscreteCollisionDetection();

//	btManifoldArray manifoldArray;
//	btBroadphasePairArray& pairArray = ghost->getOverlappingPairCache()->getOverlappingPairArray();
//
//	int numPairs = pairArray.size();
//
//	std::cout << "numPairs "<< numPairs << std::endl;
//	for (int i=0; i < numPairs; ++i)
//	{
//		manifoldArray.clear();
//		const btBroadphasePair& pair = pairArray[i];
//
//		//unless we manually perform collision detection on this pair, the contacts are in the dynamics world paircache:
//		btBroadphasePair* collisionPair = dynamicsWorld->getPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
////		btDispatcher* dispatcher = dynamicsWorld->getDispatcher()->;
//
//		std::cout << "checking pair "<< (collisionPair ? "true" : "false") << std::endl;
//		if (!collisionPair)	continue;
//
//		std::cout << "collisionPair->m_algorithm "<< (collisionPair->m_algorithm ? "true" : "false") << std::endl;
//		if (collisionPair->m_algorithm) collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
//
//		std::cout << "manifoldArray.size() "<< manifoldArray.size() << std::endl;
//		for (int j=0; j < manifoldArray.size(); ++j)
//		{
//			btPersistentManifold* manifold = manifoldArray[j];
////			btScalar directionSign = manifold->getBody0() == ghost ? btScalar(-1.0) : btScalar(1.0);
//
//			std::cout << "manifold->getNumContacts() "<< manifold->getNumContacts() << std::endl;
//			for (int p=0; p < manifold->getNumContacts(); ++p)
//			{
//				const btManifoldPoint& pt = manifold->getContactPoint(p);
//
//				std::cout << "pt.getDistance() "<< pt.getDistance() << std::endl;
//				if (pt.getDistance() < 0.0f)
//				{
////					const btVector3& ptA = pt.getPositionWorldOnA();
////					const btVector3& ptB = pt.getPositionWorldOnB();
////					const btVector3& normalOnB = pt.m_normalWorldOnB;
//
////				    auto object = ghost->getOverlappingObject(i);
//				    auto object = manifold->getBody0() == ghost ? manifold->getBody1() : manifold->getBody0();
//
//				    if (object != nullptr)
//					{
//						if (object->getUserPointer() == nullptr)
//						{
//							LOG_WARN(logger_, "Bullet collision object has a null user pointer.");
//						}
//
//						auto bulletUserData = static_cast<BulletUserData*>(object->getUserPointer());
//						std::cout << bulletUserData->type << std::endl;
//						std::cout << object->getInternalType() << std::endl;
//						switch (bulletUserData->type)
//						{
//							case BulletCollisionObjectType::RIGID_BODY:
//								result.emplace_back(bulletUserData->rigidBodyObjectHandle);
//								break;
//
//							case BulletCollisionObjectType::GHOST:
//								result.emplace_back(bulletUserData->ghostObjectHandle);
//								break;
//
//							case BulletCollisionObjectType::UNKNOWN:
//								LOG_WARN(logger_, "BulletCollisionObjectType set to Unknown");
//								break;
//
//							default:
//								LOG_WARN(logger_, "Unknown BulletCollisionObjectType");
//								break;
//						}
//					}
//
//				}
//			}
//		}
//	}

	dynamicsWorld->removeCollisionObject(&ghost);

	return result;
}

std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> Bullet::query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const float32 radius)
{
	std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> result;

	btSphereShape shape(radius);
	btPairCachingGhostObject ghost;

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(utilities::convert(origin));

	ghost.setCollisionShape(&shape);
	ghost.setWorldTransform(transform);
	ghost.setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

	auto& dynamicsWorld = physicsScenes_[physicsSceneHandle].dynamicsWorld;
	dynamicsWorld->addCollisionObject(&ghost);

	QueryContactResultCallback  resultCallback(logger_, result);

	for (int i = 0; i < ghost.getNumOverlappingObjects(); ++i)
	{
	    auto object = ghost.getOverlappingObject(i);

	    if (object != nullptr)
		{
			if (object->getUserPointer() == nullptr)
			{
				LOG_WARN(logger_, "Bullet collision object has a null user pointer.");
			}

			dynamicsWorld->contactPairTest(&ghost, object, resultCallback);
		}
	}

	dynamicsWorld->removeCollisionObject(&ghost);

	return result;
}

void Bullet::setMotionChangeListener(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, std::unique_ptr<IMotionChangeListener> motionStateListener)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	rigidBodyData->motionState->setMotionChangeListener(std::move(motionStateListener));
}

void Bullet::rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::quat& orientation)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	btTransform transform = rigidBodyData->motionState->getWorldTransform();

	const btQuaternion quat = btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
	transform.setRotation(quat);

	rigidBodyData->rigidBody->setWorldTransform(transform);
}

glm::quat Bullet::rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	const btQuaternion rotation = rigidBodyData->motionState->getWorldTransform().getRotation();

	return glm::quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
}

void Bullet::rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::quat& orientation)
{
	auto ghostData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	btTransform transform = ghostData->ghostObject->getWorldTransform();

	const btQuaternion quat = btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
	transform.setRotation(quat);

	ghostData->ghostObject->setWorldTransform(transform);
}

glm::quat Bullet::rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const
{
	auto ghostData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	const btQuaternion rotation = ghostData->ghostObject->getWorldTransform().getRotation();

	return glm::quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
}

void Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 x, const float32 y, const float32 z)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	btTransform transform = rigidBodyData->motionState->getWorldTransform();

	const btVector3 pos = btVector3(x, y, z);
	transform.setOrigin(pos);

	rigidBodyData->rigidBody->setWorldTransform(transform);
}

void Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::vec3& position)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	btTransform transform = rigidBodyData->motionState->getWorldTransform();

	const btVector3 pos = btVector3(position.x, position.y, position.z);
	transform.setOrigin(pos);

	//rigidBodyData->rigidBody->setCenterOfMassTransform(transform);
	//rigidBodyData->rigidBody->clearForces();
	//rigidBodyData->rigidBody->setLinearVelocity(btVector3(0,0,0));
	//rigidBodyData->rigidBody->setAngularVelocity(btVector3(0,0,0));

	rigidBodyData->rigidBody->setWorldTransform(transform);
	//rigidBodyData->rigidBody->getMotionState()->setWorldTransform(transform);
}

glm::vec3 Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	const btVector3 position = rigidBodyData->motionState->getWorldTransform().getOrigin();

	return glm::vec3(position.x(), position.y(), position.z());
}

void Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const float32 x, const float32 y, const float32 z)
{
	auto ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	btTransform transform = ghostObjectData->ghostObject->getWorldTransform();

	const btVector3 pos = btVector3(x, y, z);
	transform.setOrigin(pos);

	ghostObjectData->ghostObject->setWorldTransform(transform);
}

void Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::vec3& position)
{
	auto ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	btTransform transform = ghostObjectData->ghostObject->getWorldTransform();

	const btVector3 pos = btVector3(position.x, position.y, position.z);
	transform.setOrigin(pos);
	
	//ghostObjectData->ghostObject->setCenterOfMassTransform(transform);
	//ghostObjectData->ghostObject->clearForces();
	//ghostObjectData->ghostObject->setLinearVelocity(btVector3(0,0,0));
	//ghostObjectData->ghostObject->setAngularVelocity(btVector3(0,0,0));
	
	ghostObjectData->ghostObject->setWorldTransform(transform);
	//ghostObjectData->ghostObject->getMotionState()->setWorldTransform(transform);
}

glm::vec3 Bullet::position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const
{
	auto ghostObjectData = static_cast<BulletGhostObjectData*>(ghostObjectHandle.get());

	const btVector3 position = ghostObjectData->ghostObject->getWorldTransform().getOrigin();

	return glm::vec3(position.x(), position.y(), position.z());
}

void Bullet::mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 mass)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	auto shape = rigidBodyData->rigidBody->getCollisionShape();

	btScalar bulletMass(mass);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	if (bulletMass != btScalar(0.0f))
	{
		shape->calculateLocalInertia(bulletMass, localInertia);
	}

	rigidBodyData->rigidBody->setMassProps(bulletMass, localInertia);
}

float32 Bullet::mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	const auto inverseMass = rigidBodyData->rigidBody->getInvMass();
	
	return pow(inverseMass, -1.0f);
}

void Bullet::friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 friction)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	rigidBodyData->rigidBody->setFriction(btScalar(friction));
}

float32 Bullet::friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	const btScalar friction = rigidBodyData->rigidBody->getFriction();

	return friction;
}

void Bullet::restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 restitution)
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	rigidBodyData->rigidBody->setRestitution(btScalar(restitution));
}

float32 Bullet::restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const
{
	auto rigidBodyData = static_cast<BulletRigidBodyData*>(rigidBodyObjectHandle.get());

	const btScalar restitution = rigidBodyData->rigidBody->getRestitution();

	return restitution;
}

}
}
}
