#ifndef BULLET_H_
#define BULLET_H_

#include <memory>
#include <vector>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include "physics/IPhysicsEngine.hpp"
#include "physics/CollisionShapeHandle.hpp"
#include "BulletMotionState.hpp"
#include "DebugRenderer.hpp"

#include "handles/HandleVector.hpp"
#include "handles/PointerHandleVector.hpp"
#include "utilities/Properties.hpp"
#include "fs/IFileSystem.hpp"
#include "logger/ILogger.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

enum BulletCollisionObjectType
{
	UNKNOWN = 0,
	RIGID_BODY,
	GHOST
};

struct BulletUserData
{
	BulletUserData()
	{

	}

	~BulletUserData()
	{

	}

	union
	{
		RigidBodyObjectHandle rigidBodyObjectHandle;
		GhostObjectHandle ghostObjectHandle;
	};
	BulletCollisionObjectType type = BulletCollisionObjectType::UNKNOWN;
	boost::any userData;

};

struct BulletPhysicsCollisionShape
{
    std::unique_ptr<btCollisionShape> collisionShape;
};

struct BulletRigidBodyData
{
	std::unique_ptr<BulletMotionState> motionState;
	std::unique_ptr<btRigidBody> rigidBody;
	std::unique_ptr<BulletUserData> userData;
};

struct BulletGhostObjectData
{
	std::unique_ptr<btGhostObject> ghostObject;
	std::unique_ptr<BulletUserData> userData;
};

struct BulletPhysicsScene
{
	std::unique_ptr<btBroadphaseInterface> broadphase;
	std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
	std::unique_ptr<btCollisionDispatcher> dispatcher;
	std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
	std::unique_ptr<btOverlappingPairCallback> overlappingPairCallback;
	std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;

	bool debugRendering = false;

	//std::vector<std::unique_ptr<BulletRigidBodyData>> rigidBodyData;
	//std::vector<std::unique_ptr<BulletGhostObjectData>> ghostObjectData;
	handles::PointerHandleVector<BulletRigidBodyData, RigidBodyObjectHandle> rigidBodyData;
	handles::PointerHandleVector<BulletGhostObjectData, GhostObjectHandle> ghostObjectData;
};

class Bullet : public IPhysicsEngine
{
public:
	Bullet(utilities::Properties* properties, fs::IFileSystem* fileSystem, logger::ILogger* logger);
	~Bullet() override = default;

	Bullet(const Bullet& other) = delete;
    Bullet& operator=(const Bullet& other) = delete;

	void tick(const PhysicsSceneHandle& physicsSceneHandle, const float32 delta) override;
    void renderDebug(const PhysicsSceneHandle& physicsSceneHandle) override;

    PhysicsSceneHandle createPhysicsScene() override;
	void destroy(const PhysicsSceneHandle& physicsSceneHandle) override;

	void setGravity(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& gravity) override;

	void setPhysicsDebugRenderer(IPhysicsDebugRenderer* physicsDebugRenderer) override;
	void setDebugRendering(const PhysicsSceneHandle& physicsSceneHandle, const bool enabled) override;

	CollisionShapeHandle createStaticPlaneShape(const glm::vec3& planeNormal, const float32 planeConstant) override;
	CollisionShapeHandle createStaticBoxShape(const glm::vec3& dimensions) override;
    CollisionShapeHandle createStaticSphereShape(const float32 radius) override;
    CollisionShapeHandle createStaticTerrainShape(const IHeightfield& heightfield) override;
	void destroy(const CollisionShapeHandle& collisionShapeHandle) override;
	void destroyAllStaticShapes() override;

	RigidBodyObjectHandle createRigidBodyObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr,
		const boost::any& userData = boost::any()
	) override;
	RigidBodyObjectHandle createRigidBodyObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		const float32 mass,
		const float32 friction,
		const float32 restitution,
		std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr,
		const boost::any& userData = boost::any()
	) override;
	RigidBodyObjectHandle createRigidBodyObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		const glm::vec3& position,
		const glm::quat& orientation,
		const float32 mass = 1.0f,
		const float32 friction = 1.0f,
		const float32 restitution = 1.0f,
		std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr,
		const boost::any& userData = boost::any()
	) override;
	GhostObjectHandle createGhostObject(const PhysicsSceneHandle& physicsSceneHandle, const CollisionShapeHandle& collisionShapeHandle, const boost::any& userData = boost::any()) override;
	GhostObjectHandle createGhostObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		const glm::vec3& position,
		const glm::quat& orientation,
		const boost::any& userData = boost::any()
	) override;
	void destroy(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) override;
	void destroy(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) override;
	void destroyAllRigidBodies() override;

	void setUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const boost::any& userData) override;
	void setUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const boost::any& userData) override;
	boost::any& getUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;
	boost::any& getUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	Raycast raycast(const PhysicsSceneHandle& physicsSceneHandle, const ray::Ray& ray) override;

	std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const std::vector<glm::vec3>& points) final;
	std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const float32 radius) override;

	void setMotionChangeListener(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, std::unique_ptr<IMotionChangeListener> motionStateListener) override;

	void rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::quat& orientation) override;
	glm::quat rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	void rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::quat& orientation) override;
	glm::quat rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	void position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 x, const float32 y, const float32 z) override;
	void position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::vec3& position) override;
	glm::vec3 position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	void position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const float32 x, const float32 y, const float32 z) override;
	void position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::vec3& position) override;
	glm::vec3 position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	void mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 mass) override;
	float32 mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	void friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 friction) override;
	float32 friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;
	
	void restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 restitution) override;
	float32 restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

private:
    utilities::Properties* properties_;
	fs::IFileSystem* fileSystem_;
	logger::ILogger* logger_;

    handles::HandleVector<BulletPhysicsCollisionShape, CollisionShapeHandle> shapes_;
	//handles::HandleVector<BulletRigidBodyData, RigidBodyObjectHandle> rigidBodyData_;
	//handles::HandleVector<BulletGhostObjectData, GhostObjectHandle> ghostObjectData_;
	handles::HandleVector<BulletPhysicsScene, PhysicsSceneHandle> physicsScenes_;

	std::unique_ptr<DebugRenderer> debugRenderer_;
};

}
}
}

#endif /* BULLET_H_ */
