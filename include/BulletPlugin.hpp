#ifndef BULLETPLUGIN_H_
#define BULLETPLUGIN_H_

#include <memory>

#include "IPhysicsPlugin.hpp"

namespace ice_engine
{

class BulletPlugin : public IPhysicsPlugin
{
public:
	BulletPlugin() = default;
	virtual ~BulletPlugin() override = default;

	virtual std::string getName() const override;

	virtual std::unique_ptr<physics::IPhysicsEngineFactory> createFactory() const override;

};

}

#endif /* BULLETPLUGIN_H_ */
