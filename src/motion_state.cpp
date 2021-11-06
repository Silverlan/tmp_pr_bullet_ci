/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "motion_state.hpp"
#include "environment.hpp"

KinematicMotionState::KinematicMotionState(pragma::physics::BtCollisionObject &o,const umath::Transform &initialTransform)
	: SimpleMotionState{o},m_transform{initialTransform}
{}
KinematicMotionState::~KinematicMotionState() {}
void KinematicMotionState::getWorldTransform(btTransform &worldTrans) const {worldTrans = pragma::physics::BtEnvironment::CreateBtTransform(m_transform);}
void KinematicMotionState::setWorldTransform(const btTransform &worldTrans) {SimpleMotionState::setWorldTransform(worldTrans);/*m_transform.SetTransform(worldTrans);*/}
umath::Transform &KinematicMotionState::GetWorldTransform() {return m_transform;}
const umath::Transform &KinematicMotionState::GetWorldTransform() const {return const_cast<KinematicMotionState*>(this)->GetWorldTransform();}
