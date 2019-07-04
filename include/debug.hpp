#ifndef __PR_BT_DEBUG_HPP__
#define __PR_BT_DEBUG_HPP__

#include "common.hpp"
#include <pragma/physics/visual_debugger.hpp>

namespace pragma::physics
{
	class BtVisualDebugger
		: public IVisualDebugger,
		public btIDebugDraw
	{
	public:
		virtual void drawLine(const btVector3 &from,const btVector3 &to,const btVector3 &color) override;
		virtual void drawLine(const btVector3 &from,const btVector3 &to,const btVector3 &fromColor,const btVector3 &toColor) override;
		virtual void drawContactPoint(const btVector3 &PointOnB,const btVector3 &normalOnB,btScalar distance,int lifeTime,const btVector3 &color) override;
		virtual void reportErrorWarning(const char *warningString) override;
		virtual void draw3dText(const btVector3 &location,const char *textString) override;
		virtual void setDebugMode(int debugMode) override;
		virtual int getDebugMode() const override;
	private:
		int m_debugMode = 0;
	};
};

#endif
