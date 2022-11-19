cmake_minimum_required(VERSION 3.12)

include("./CMakeShared.txt")

set(PROJ_NAME pr_bullet)

project(${PROJ_NAME} CXX)



set(DEPENDENCY_BULLET_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/third_party/bullet/src CACHE PATH "Path to include directory." FORCE)
#search_lib_recursive(DEPENDENCY_BULLET_LIBRARY third_party/bullet/src/${BINARY_PRAGMA_DIR} bullet STATIC)
search_lib_recursive(DEPENDENCY_BULLET_LINEAR_MATH_LIBRARY third_party/bullet/src/LinearMath/${BINARY_PRAGMA_DIR} LinearMath STATIC)
search_lib_recursive(DEPENDENCY_BULLET_DYNAMICS_LIBRARY third_party/bullet/src/BulletDynamics/${BINARY_PRAGMA_DIR} BulletDynamics STATIC)
search_lib_recursive(DEPENDENCY_BULLET_COLLISION_LIBRARY third_party/bullet/src/BulletCollision/${BINARY_PRAGMA_DIR} BulletCollision STATIC)
search_lib_recursive(DEPENDENCY_BULLET_SOFTBODY_LIBRARY third_party/bullet/src/BulletSoftBody/${BINARY_PRAGMA_DIR} BulletSoftBody STATIC)
# set(PRECOMPILED_HEADER "stdafx")

add_external_library(bullet_dynamics)
add_external_library(bullet_collision)
add_external_library(bullet_softbody)
add_external_library(bullet_linear_math)
add_include_dir(bullet)
add_include_dir(glm)

# required for bullet
add_def(_ENABLE_EXTENDED_ALIGNED_STORAGE)

add_def(BT_USE_DOUBLE_PRECISION)

link_generic_module_libraries()
register_pragma_module(${PROJ_NAME})

#


if(UNIX)
#	resolve_links(DEPENDENCY_BULLET_COLLISION_LIBRARY)
#	resolve_links(DEPENDENCY_BULLET_DYNAMICS_LIBRARY)
#	resolve_links(DEPENDENCY_BULLET_LIBRARY)
#	resolve_links(DEPENDENCY_BULLET_LINEAR_MATH_LIBRARY)
#	resolve_links(DEPENDENCY_BULLET_SOFTBODY_LIBRARY)
	SET(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
endif()

# bullet
set(USE_DOUBLE_PRECISION ON CACHE BOOL ON FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL OFF FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL OFF FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL OFF FORCE)
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "" FORCE)
# register_third_party_library_custom(bullet third_party)
add_subdirectory(third_party/bullet)
set(BUILD_SHARED_LIBS ON CACHE BOOL ON FORCE)
set_target_properties(App_obj2sdf PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet2FileLoader PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet3Collision PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet3Common PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet3Dynamics PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet3Geometry PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(Bullet3OpenCL_clew PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletCollision PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletDynamics PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletFileLoader PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletInverseDynamics PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletInverseDynamicsUtils PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletRobotics PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletSoftBody PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletWorldImporter PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(BulletXmlWorldImporter PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(ConvexDecomposition PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(GIMPACTUtils PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(HACD PROPERTIES FOLDER modules/physics/bullet/bullet)
set_target_properties(LinearMath PROPERTIES FOLDER modules/physics/bullet/bullet)

if(WIN32)
	target_compile_options(Bullet2FileLoader PUBLIC "/MD")
	target_compile_options(Bullet3Collision PUBLIC "/MD")
	target_compile_options(Bullet3Common PUBLIC "/MD")
	target_compile_options(Bullet3Dynamics PUBLIC "/MD")
	target_compile_options(Bullet3Geometry PUBLIC "/MD")
	target_compile_options(Bullet3OpenCL_clew PUBLIC "/MD")
	target_compile_options(BulletCollision PUBLIC "/MD")
	target_compile_options(BulletDynamics PUBLIC "/MD")
	target_compile_options(BulletFileLoader PUBLIC "/MD")
	target_compile_options(BulletInverseDynamics PUBLIC "/MD")
	target_compile_options(BulletInverseDynamicsUtils PUBLIC "/MD")
	target_compile_options(BulletRobotics PUBLIC "/MD")
	target_compile_options(BulletSoftBody PUBLIC "/MD")
	target_compile_options(BulletWorldImporter PUBLIC "/MD")
	target_compile_options(BulletXmlWorldImporter PUBLIC "/MD")
	target_compile_options(LinearMath PUBLIC "/MD")
endif()

add_dependencies(BulletCollision LinearMath)
add_dependencies(BulletDynamics BulletCollision)
add_dependencies(pr_bullet BulletDynamics BulletSoftBody)

#

set_target_properties(pr_bullet PROPERTIES FOLDER modules/physics/bullet)

set_property(GLOBAL PROPERTY PRAGMA_MODULE_SKIP_TARGET_PROPERTY_FOLDER 1)
