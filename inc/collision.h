//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_COLLISION_H
#define	UTIL_C_CRT_COLLISION_H

#include "geometry_def.h"

typedef struct CCTRigidBody_t {
	CCTNum_t velocity[3];
	CCTNum_t mass;
	CCTNum_t cor;
} CCTRigidBody_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll CCTRigidBody_t* physRigidBodyInit(CCTRigidBody_t* rigid);
__declspec_dll void physCollisionResolveVelocity(const CCTRigidBody_t* src_rd, const CCTRigidBody_t* target_rd, const CCTNum_t hit_normal[3], CCTNum_t src_v[3], CCTNum_t target_v[3]);
__declspec_dll CCTNum_t* physCollisionInertiaTensor(const unsigned char* geo_data, int geo_type, CCTNum_t mat44[16]);

#ifdef	__cplusplus
}
#endif

#endif
