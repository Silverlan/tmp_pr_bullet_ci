#ifndef __PHYS_KINEMATIC_MOTION_STATE_HPP__
#define __PHYS_KINEMATIC_MOTION_STATE_HPP__

#include "pragma/physics/transform.hpp"
#include <LinearMath/btMotionState.h>

class KinematicMotionState
	: public btMotionState
{
public:
	KinematicMotionState(const pragma::physics::Transform &initialTransform={});
	virtual ~KinematicMotionState() override;

	pragma::physics::Transform &GetWorldTransform();
	const pragma::physics::Transform &GetWorldTransform() const;
private:
	virtual void getWorldTransform(btTransform &worldTrans) const override;
	virtual void setWorldTransform(const btTransform &worldTrans) override;
	pragma::physics::Transform m_transform = {};
};

#endif
