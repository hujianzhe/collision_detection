//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_COLLISION_H
#define	UTIL_C_CRT_COLLISION_H

#include "geometry_def.h"

typedef struct CCTResult_t {
	CCTNum_t distance;
	CCTNum_t hit_normal[3];

	int has_unique_hit_point;
	CCTNum_t unique_hit_point[3];
} CCTResult_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll GeometryBody_t* mathGeometryBodyClone(GeometryBody_t* dst, const unsigned char* geo_data, int geo_type);
__declspec_dll void mathGeometryBodyFreeData(GeometryBody_t* b);
__declspec_dll void mathGeometryBodyRefFreeData(GeometryBodyRef_t* b);

__declspec_dll const CCTNum_t* mathGeometryBodyPosition(const GeometryBodyRef_t* b);
__declspec_dll void mathGeometryBodySetPosition(GeometryBodyRef_t* b, const CCTNum_t v[3]);

__declspec_dll GeometryAABB_t* mathCollisionBodyBoundingBox(const GeometryBodyRef_t* b, GeometryAABB_t* aabb);
__declspec_dll int mathCollisionBodyRotate(GeometryBodyRef_t* b, const CCTNum_t q[4]);
__declspec_dll int mathCollisionBodyRotateAxisRadian(GeometryBodyRef_t* b, const CCTNum_t axis[3], CCTNum_t radian);

__declspec_dll int mathCollisionContain(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll int mathCollisionIntersect(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll CCTResult_t* mathCollisionSweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTResult_t* result);

#ifdef	__cplusplus
}
#endif

#endif
