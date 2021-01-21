#ifndef PHYSICSDEBUGRENDERER_H_
#define PHYSICSDEBUGRENDERER_H_

#include <LinearMath/btIDebugDraw.h>

#include "physics/IPhysicsDebugRenderer.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

class DebugRenderer : public btIDebugDraw
{
public:
	DebugRenderer(physics::IPhysicsDebugRenderer* physicsDebugRenderer) : physicsDebugRenderer_(physicsDebugRenderer)
	{
	}
	
	~DebugRenderer() override = default;
	
	void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override
	{
		physicsDebugRenderer_->pushLine(glm::vec3(from.x(), from.y(), from.z()), glm::vec3(to.x(), to.y(), to.z()), glm::vec3(color.x(), color.y(), color.z()));
	};
	
	void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override {};
	void reportErrorWarning(const char* warningString) override {};
	void draw3dText(const btVector3& location, const char* textString) override {};
	void setDebugMode(int debugMode) override
	{
		debugDrawMode_ = debugMode;
	};
	
	int getDebugMode() const override
	{
		return debugDrawMode_;
	};
	
private:
	IPhysicsDebugRenderer* physicsDebugRenderer_ = nullptr;
	int debugDrawMode_ =  DBG_MAX_DEBUG_DRAW_MODE; //DBG_DrawWireframe | DBG_DrawAabb;

};

}
}
}

#endif /* PHYSICSDEBUGRENDERER_H_ */
