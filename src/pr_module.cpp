#include "pr_module.hpp"
#include "environment.hpp"
#include <sharedutils/util_weak_handle.hpp>
#include <mathutil/umath.h>
#include <iostream>
#include <array>

extern "C"
{
	PRAGMA_EXPORT void initialize_physics_engine(NetworkState &nw,std::unique_ptr<pragma::physics::IEnvironment> &outEnv)
	{
		outEnv = std::make_unique<pragma::physics::BtEnvironment>(nw);
		if(outEnv->Initialize() == false)
			outEnv = nullptr;
	}
};
