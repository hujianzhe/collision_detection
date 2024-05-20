//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_COLLISION_H
#define	UTIL_C_CRT_COLLISION_H

#include "geometry_def.h"
#include <stddef.h>

typedef struct CCTResult_t {
	CCTNum_t distance;
	CCTNum_t hit_normal[3];

	int has_unique_hit_point;
	CCTNum_t unique_hit_point[3];
} CCTResult_t;

typedef struct CCTRigidBody_t {
	CCTNum_t velocity[3];
	CCTNum_t mass;
	CCTNum_t cor;
} CCTRigidBody_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll int mathCollisionContain(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll int mathCollisionIntersect(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll CCTResult_t* mathCollisionSweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTResult_t* result);

__declspec_dll CCTRigidBody_t* physRigidBodyInit(CCTRigidBody_t* rigid);
__declspec_dll void physCollisionResolveVelocity(const CCTRigidBody_t* src_rd, const CCTRigidBody_t* target_rd, const CCTNum_t hit_normal[3], CCTNum_t src_v[3], CCTNum_t target_v[3]);

#ifdef	__cplusplus
}
#endif

#endif
