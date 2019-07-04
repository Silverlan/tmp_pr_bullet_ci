#include "motion_state.hpp"
#include "environment.hpp"

KinematicMotionState::KinematicMotionState(const pragma::physics::Transform &initialTransform)
	: m_transform{initialTransform}
{}
KinematicMotionState::~KinematicMotionState() {}
void KinematicMotionState::getWorldTransform(btTransform &worldTrans) const {worldTrans = pragma::physics::BtEnvironment::CreateBtTransform(m_transform);}
void KinematicMotionState::setWorldTransform(const btTransform &worldTrans) {/*m_transform.SetTransform(worldTrans);*/}
pragma::physics::Transform &KinematicMotionState::GetWorldTransform() {return m_transform;}
const pragma::physics::Transform &KinematicMotionState::GetWorldTransform() const {return const_cast<KinematicMotionState*>(this)->GetWorldTransform();}
