//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_COLLISION_H
#define	UTIL_C_CRT_COLLISION_H

#include "geometry_def.h"

typedef struct CCTResult_t {
	CCTNum_t distance;
	int hit_point_cnt;
	CCTNum_t hit_point[3];
	CCTNum_t hit_normal[3];
} CCTResult_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll GeometryAABB_t* mathCollisionBodyBoundingBox(const GeometryBodyRef_t* b, GeometryAABB_t* aabb);
__declspec_dll int mathCollisionBodyRotate(GeometryBodyRef_t* b, const CCTNum_t mark_pos[3], const CCTNum_t q[4]);
__declspec_dll int mathCollisionBodyRotateAxisRadian(GeometryBodyRef_t* b, const CCTNum_t mark_pos[3], const CCTNum_t axis[3], CCTNum_t radian);

__declspec_dll int mathCollisionContain(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll int mathCollisionIntersect(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll CCTResult_t* mathCollisionSweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTResult_t* result);

#ifdef	__cplusplus
}
#endif

#endif
