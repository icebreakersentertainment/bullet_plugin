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
	virtual ~Bullet() override = default;

	Bullet(const Bullet& other) = delete;

	virtual void tick(const PhysicsSceneHandle& physicsSceneHandle, const float32 delta) override;

	virtual PhysicsSceneHandle createPhysicsScene() override;
	virtual void destroyPhysicsScene(const PhysicsSceneHandle& physicsSceneHandle) override;

	virtual void setGravity(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& gravity) override;

	virtual void setPhysicsDebugRenderer(IPhysicsDebugRenderer* physicsDebugRenderer) override;
	virtual void setDebugRendering(const PhysicsSceneHandle& physicsSceneHandle, const bool enabled) override;

	virtual CollisionShapeHandle createStaticPlaneShape(const glm::vec3& planeNormal, const float32 planeConstant) override;
	virtual CollisionShapeHandle createStaticBoxShape(const glm::vec3& dimensions) override;
	virtual CollisionShapeHandle createStaticTerrainShape(const IHeightfield* heightfield) override;
	virtual void destroyStaticShape(const CollisionShapeHandle& collisionShapeHandle) override;
	virtual void destroyAllStaticShapes() override;

	virtual RigidBodyObjectHandle createRigidBodyObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr,
		const boost::any& userData = boost::any()
	) override;
	virtual RigidBodyObjectHandle createRigidBodyObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		const float32 mass,
		const float32 friction,
		const float32 restitution,
		std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr,
		const boost::any& userData = boost::any()
	) override;
	virtual RigidBodyObjectHandle createRigidBodyObject(
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
	virtual GhostObjectHandle createGhostObject(const PhysicsSceneHandle& physicsSceneHandle, const CollisionShapeHandle& collisionShapeHandle, const boost::any& userData = boost::any()) override;
	virtual GhostObjectHandle createGhostObject(
		const PhysicsSceneHandle& physicsSceneHandle,
		const CollisionShapeHandle& collisionShapeHandle,
		const glm::vec3& position,
		const glm::quat& orientation,
		const boost::any& userData = boost::any()
	) override;
	virtual void destroy(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) override;
	virtual void destroy(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) override;
	virtual void destroyAllRigidBodies() override;

	virtual void setUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const boost::any& userData) override;
	virtual void setUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const boost::any& userData) override;
	virtual boost::any& getUserData(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;
	virtual boost::any& getUserData(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	virtual Raycast raycast(const PhysicsSceneHandle& physicsSceneHandle, const ray::Ray& ray) override;

	virtual std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const std::vector<glm::vec3>& points) final;
	virtual std::vector<boost::variant<RigidBodyObjectHandle, GhostObjectHandle>> query(const PhysicsSceneHandle& physicsSceneHandle, const glm::vec3& origin, const float32 radius) override;

	virtual void setMotionChangeListener(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, std::unique_ptr<IMotionChangeListener> motionStateListener) override;

	virtual void rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::quat& orientation) override;
	virtual glm::quat rotation(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	virtual void rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::quat& orientation) override;
	virtual glm::quat rotation(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	virtual void position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 x, const float32 y, const float32 z) override;
	virtual void position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const glm::vec3& position) override;
	virtual glm::vec3 position(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	virtual void position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const float32 x, const float32 y, const float32 z) override;
	virtual void position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle, const glm::vec3& position) override;
	virtual glm::vec3 position(const PhysicsSceneHandle& physicsSceneHandle, const GhostObjectHandle& ghostObjectHandle) const override;

	virtual void mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 mass) override;
	virtual float32 mass(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

	virtual void friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 friction) override;
	virtual float32 friction(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;
	
	virtual void restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle, const float32 restitution) override;
	virtual float32 restitution(const PhysicsSceneHandle& physicsSceneHandle, const RigidBodyObjectHandle& rigidBodyObjectHandle) const override;

private:
	utilities::Properties* properties_;
	fs::IFileSystem* fileSystem_;
	logger::ILogger* logger_;



	std::vector<std::unique_ptr<btCollisionShape>> shapes_;
	//handles::HandleVector<BulletRigidBodyData, RigidBodyObjectHandle> rigidBodyData_;
	//handles::HandleVector<BulletGhostObjectData, GhostObjectHandle> ghostObjectData_;
	handles::HandleVector<BulletPhysicsScene, PhysicsSceneHandle> physicsScenes_;

	std::unique_ptr<DebugRenderer> debugRenderer_;
};

}
}
}

#endif /* BULLET_H_ */
