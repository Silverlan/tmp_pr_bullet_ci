#include "material.hpp"
#include "environment.hpp"

pragma::physics::BtMaterial::BtMaterial(BtEnvironment &env,float staticFriction,float dynamicFriction,float restitution)
	: IMaterial{env},m_staticFriction{staticFriction},m_dynamicFriction{dynamicFriction},m_restitution{restitution}
{}
float pragma::physics::BtMaterial::GetStaticFriction() const {return m_staticFriction;}
void pragma::physics::BtMaterial::SetStaticFriction(float friction) {m_staticFriction = friction;}
float pragma::physics::BtMaterial::GetDynamicFriction() const {return m_dynamicFriction;}
void pragma::physics::BtMaterial::SetDynamicFriction(float friction) {m_dynamicFriction = friction;}
float pragma::physics::BtMaterial::GetRestitution() const {return m_restitution;}
void pragma::physics::BtMaterial::SetRestitution(float restitution) {m_restitution = restitution;}
