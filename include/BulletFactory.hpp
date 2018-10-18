#ifndef BULLETFACTORY_H_
#define BULLETFACTORY_H_

#include <memory>

#include "physics/IPhysicsEngineFactory.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

class BulletFactory : public IPhysicsEngineFactory
{
public:
	BulletFactory() = default;
	virtual ~BulletFactory() override = default;

	virtual std::unique_ptr<IPhysicsEngine> create(
		utilities::Properties* properties,
		fs::IFileSystem* fileSystem,
		logger::ILogger* logger
	) override;

};

}
}
}

#endif /* BULLETFACTORY_H_ */
