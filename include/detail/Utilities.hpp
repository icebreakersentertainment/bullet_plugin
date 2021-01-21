#ifndef BULLETPHYSICSUTILITIES_H_
#define BULLETPHYSICSUTILITIES_H_

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace ice_engine
{
namespace physics
{
namespace bullet
{
namespace detail
{

// Bullet to GLM
inline glm::vec3 toGlm(const btVector3& bulletVector3)
{
	return glm::vec3(bulletVector3.x(), bulletVector3.y(), bulletVector3.z());
};

inline glm::quat toGlm(const btQuaternion& bulletQuaternion)
{
	return glm::quat(bulletQuaternion.w(), bulletQuaternion.x(), bulletQuaternion.y(), bulletQuaternion.z());
};

// GLM to Bullet
inline btVector3 toBullet(const glm::vec3& glmVector3)
{
	return btVector3(glmVector3.x, glmVector3.y, glmVector3.z);
};

inline btQuaternion toBullet(const glm::quat& glmQuat)
{
	return btQuaternion(glmQuat.x, glmQuat.y, glmQuat.z, glmQuat.w);
};

}
}
}
}

#endif /* BULLETPHYSICSUTILITIES_H_ */
