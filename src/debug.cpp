#include "debug.hpp"
#include "environment.hpp"

#pragma optimize("",off)
void pragma::physics::BtVisualDebugger::setDebugMode(int debugMode)
{
	Clear();
	m_debugMode = debugMode;
	if(m_debugMode == btIDebugDraw::DBG_NoDebug)
		return;
	Draw();
}

void pragma::physics::BtVisualDebugger::drawLine(const btVector3 &from,const btVector3 &to,const btVector3 &color)
{
	drawLine(from,to,color,color);
}
void pragma::physics::BtVisualDebugger::drawLine(const btVector3 &from,const btVector3 &to,const btVector3 &fromColor,const btVector3 &toColor)
{
	DrawLine(
		Vector3(from.x() /BtEnvironment::WORLD_SCALE,from.y() /BtEnvironment::WORLD_SCALE,from.z() /BtEnvironment::WORLD_SCALE),
		Vector3(to.x() /BtEnvironment::WORLD_SCALE,to.y() /BtEnvironment::WORLD_SCALE,to.z() /BtEnvironment::WORLD_SCALE),
		Color(static_cast<int16_t>(fromColor.x() *255.f),static_cast<int16_t>(fromColor.y() *255.f),static_cast<int16_t>(fromColor.z() *255.f)),
		Color(static_cast<int16_t>(toColor.x() *255.f),static_cast<int16_t>(toColor.y() *255.f),static_cast<int16_t>(toColor.z() *255.f))
	);
}
void pragma::physics::BtVisualDebugger::drawContactPoint(const btVector3 &pointOnB,const btVector3 &normalOnB,btScalar dist,int lifeTime,const btVector3 &color)
{
	DrawContactPoint(
		Vector3(pointOnB.x() /BtEnvironment::WORLD_SCALE,pointOnB.y() /BtEnvironment::WORLD_SCALE,pointOnB.z() /BtEnvironment::WORLD_SCALE),
		Vector3{normalOnB.x(),normalOnB.y(),normalOnB.z()},
		dist,lifeTime,
		Color(static_cast<int16_t>(color.x() *255.f),static_cast<int16_t>(color.y() *255.f),static_cast<int16_t>(color.z() *255.f))
	);
}
void pragma::physics::BtVisualDebugger::reportErrorWarning(const char *warningString)
{
	Con::cwar<<"[BULLET] "<<warningString<<Con::endl;
	ReportErrorWarning(warningString);
};
void pragma::physics::BtVisualDebugger::draw3dText(const btVector3 &pos,const char *str)
{
	Draw3DText(Vector3(pos.x() /BtEnvironment::WORLD_SCALE,pos.y() /BtEnvironment::WORLD_SCALE,pos.z() /BtEnvironment::WORLD_SCALE),str);
}
int pragma::physics::BtVisualDebugger::getDebugMode() const {return m_debugMode;}
#pragma optimize("",on)
