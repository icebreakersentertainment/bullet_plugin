#include <boost/config.hpp> // for BOOST_SYMBOL_EXPORT

#include "BulletPlugin.hpp"

#include "BulletFactory.hpp"

namespace ice_engine
{

std::string BulletPlugin::getName() const
{
	return std::string("bullet");
}

std::unique_ptr<physics::IPhysicsEngineFactory> BulletPlugin::createFactory() const
{
	std::unique_ptr<physics::IPhysicsEngineFactory> ptr = std::make_unique< physics::bullet::BulletFactory >();
	
	return std::move( ptr );
}

// Exporting `my_namespace::plugin` variable with alias name `plugin`
// (Has the same effect as `BOOST_DLL_ALIAS(my_namespace::plugin, plugin)`)
extern "C" BOOST_SYMBOL_EXPORT BulletPlugin plugin;
BulletPlugin plugin;

}
