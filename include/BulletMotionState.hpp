#ifndef BULLETMOTIONSTATE_H_
#define BULLETMOTIONSTATE_H_

#include <btBulletDynamicsCommon.h>

#include "physics/IMotionChangeListener.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

class BulletMotionState : public btMotionState
{
public:
	BulletMotionState(const btTransform& initialPosition, std::unique_ptr<IMotionChangeListener> motionStateListener = nullptr)
		:
		initialPosition_(initialPosition),
		motionStateListener_(std::move(motionStateListener))
	{
	}

	virtual ~BulletMotionState() override = default;
	
	void setMotionChangeListener(std::unique_ptr<IMotionChangeListener> motionStateListener)
	{
		motionStateListener_ = std::move(motionStateListener);
	}
	
	btTransform getWorldTransform() const
	{
		return initialPosition_;
	}
	
	virtual void getWorldTransform(btTransform& worldTrans) const override
	{
		worldTrans = initialPosition_;
	}

	virtual void setWorldTransform(const btTransform& worldTrans) override
	{
		if (motionStateListener_.get() == nullptr)
		{
			return;
		}
		
		initialPosition_ = worldTrans;

		const btQuaternion rot = worldTrans.getRotation();
		const btVector3 pos = worldTrans.getOrigin();
		
		motionStateListener_->update(glm::vec3(pos.x(), pos.y(), pos.z()), glm::quat(rot.w(), rot.x(), rot.y(), rot.z()));
	}

private:
	btTransform initialPosition_;
	std::unique_ptr<IMotionChangeListener> motionStateListener_;
};

}
}
}

#endif /* BULLETMOTIONSTATE_H_ */
