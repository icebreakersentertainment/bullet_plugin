#include "BulletFactory.hpp"

#include "Bullet.hpp"

namespace ice_engine
{
namespace physics
{
namespace bullet
{

std::unique_ptr<IPhysicsEngine> BulletFactory::create(
        utilities::Properties* properties,
        fs::IFileSystem* fileSystem,
        logger::ILogger* logger
)
{
	std::unique_ptr<IPhysicsEngine> ptr = std::make_unique< Bullet >(
		properties,
		fileSystem,
		logger
	);
	
	return std::move( ptr );
}

}
}
}
