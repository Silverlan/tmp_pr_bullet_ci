cmake_minimum_required(VERSION 3.12)

include("../CMakeShared.txt")

set(PROJ_NAME pr_bullet)

project(${PROJ_NAME} CXX)

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
