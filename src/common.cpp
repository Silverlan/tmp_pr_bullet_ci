#include "common.hpp"

btVector3 uvec::create_bt(const Vector3 &v) {return btVector3{v.x,v.y,v.z};}
btQuaternion uquat::create_bt(const Quat &v) {return btQuaternion{v.x,v.y,v.z,v.w};}

Vector3 uvec::create(const btVector3 &v) {return Vector3{static_cast<float>(v.x()),static_cast<float>(v.y()),static_cast<float>(v.z())};}
Quat uquat::create(const btQuaternion &v) {return Quat{static_cast<float>(v.w()),static_cast<float>(v.x()),static_cast<float>(v.y()),static_cast<float>(v.z())};}
