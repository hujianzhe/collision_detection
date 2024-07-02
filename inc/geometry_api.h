//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_API_H
#define	UTIL_C_CRT_GEOMETRY_API_H

#include "geometry_def.h"
#include <stddef.h>

typedef struct CCTSweepResult_t {
	CCTNum_t distance;
	CCTNum_t hit_plane_v[3];
	CCTNum_t hit_plane_n[3];
	int hit_point_unique;
} CCTSweepResult_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll size_t mathGeometrySize(int geo_type);

__declspec_dll unsigned char* mathGeometryClone(unsigned char* dst_geo_data, const unsigned char* src_geo_data, int src_geo_type);
__declspec_dll void mathGeometryFree(void* geo_data, int geo_type);
__declspec_dll void mathGeometryFreeBody(GeometryBody_t* b);
__declspec_dll void mathGeometryFreeRef(GeometryBodyRef_t* b);

__declspec_dll const CCTNum_t* mathGeometryGetPosition(const GeometryBodyRef_t* b, CCTNum_t v[3]);
__declspec_dll void mathGeometrySetPosition(GeometryBodyRef_t* b, const CCTNum_t v[3]);

__declspec_dll GeometryAABB_t* mathGeometryBoundingBox(const GeometryBodyRef_t* b, GeometryAABB_t* aabb);
__declspec_dll int mathGeometryRotate(GeometryBodyRef_t* b, const CCTNum_t q[4]);
__declspec_dll int mathGeometryRotateAxisRadian(GeometryBodyRef_t* b, const CCTNum_t axis[3], CCTNum_t radian);

__declspec_dll int mathGeometryContain(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll int mathGeometryIntersect(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two);
__declspec_dll CCTSweepResult_t* mathGeometrySweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTSweepResult_t* result);

#ifdef	__cplusplus
}
#endif

#endif
