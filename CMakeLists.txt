include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME pr_bullet)
pr_add_library(${PROJ_NAME} SHARED)

pr_add_dependency(${PROJ_NAME} shared TARGET PUBLIC)

pr_add_headers(${PROJ_NAME} "include/")
pr_add_sources(${PROJ_NAME} "src/")

pr_add_compile_definitions(${PROJ_NAME} -D_ENABLE_EXTENDED_ALIGNED_STORAGE -DBT_USE_DOUBLE_PRECISION PUBLIC)

set(PKGCONFIG_INSTALL_PREFIX ".")

# Bullet
set(USE_DOUBLE_PRECISION ON CACHE BOOL ON FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL OFF FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL OFF FORCE)
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "" FORCE)
SET(PKGCONFIG_INSTALL_PREFIX "lib${LIB_SUFFIX}/pkgconfig/" CACHE STRING "Base directory for pkgconfig files" FORCE)
add_subdirectory(third_party/bullet)
set(BUILD_SHARED_LIBS ON CACHE BOOL ON FORCE)

target_include_directories(${PROJ_NAME} PUBLIC third_party/bullet/src/)
add_dependencies(BulletCollision LinearMath)
add_dependencies(BulletDynamics BulletCollision)

set(BT_TARGETS
	App_obj2sdf
	Bullet2FileLoader
	Bullet3Collision
	Bullet3Common
	Bullet3Dynamics
	Bullet3Geometry
	Bullet3OpenCL_clew
	BulletCollision
	BulletDynamics
	BulletFileLoader
	BulletInverseDynamics
	BulletInverseDynamicsUtils
	BulletRobotics
	BulletSoftBody
	BulletWorldImporter
	BulletXmlWorldImporter
	ConvexDecomposition
	GIMPACTUtils
	HACD
	LinearMath
)

if(UNIX)
	foreach(target ${BT_TARGETS})
		if(TARGET ${target})
			set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
		endif()
	endforeach()
elseif(WIN32)
	foreach(target ${BT_TARGETS})
		if(TARGET ${target})
			target_compile_options(${target} PUBLIC "/MD")
			set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
		endif()
	endforeach()
endif()

foreach(target ${BT_TARGETS})
	if(TARGET ${target})
		set_target_properties(${target} PROPERTIES FOLDER modules/physics/bullet/bullet)
	endif()
endforeach()

pr_add_dependency(${PROJ_NAME} LinearMath TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} BulletDynamics TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} BulletCollision TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} BulletSoftBody TARGET PUBLIC)

pr_finalize(${PROJ_NAME} FOLDER "modules/physics/bullet")
