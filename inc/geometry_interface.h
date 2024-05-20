//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_INTERFACE_H
#define	UTIL_C_CRT_GEOMETRY_INTERFACE_H

#include "geometry_def.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll size_t mathGeometrySize(int geo_type);
__declspec_dll unsigned char* mathGeometryClone(unsigned char* dst_geo_data, const unsigned char* src_geo_data, int src_geo_type);
__declspec_dll void mathGeometryFreeData(GeometryBody_t* b);
__declspec_dll void mathGeometryRefFreeData(GeometryBodyRef_t* b);

__declspec_dll const CCTNum_t* mathGeometryGetPosition(const GeometryBodyRef_t* b, CCTNum_t v[3]);
__declspec_dll void mathGeometrySetPosition(GeometryBodyRef_t* b, const CCTNum_t v[3]);

__declspec_dll GeometryAABB_t* mathGeometryBoundingBox(const GeometryBodyRef_t* b, GeometryAABB_t* aabb);
__declspec_dll int mathGeometryRotate(GeometryBodyRef_t* b, const CCTNum_t q[4]);
__declspec_dll int mathGeometryRotateAxisRadian(GeometryBodyRef_t* b, const CCTNum_t axis[3], CCTNum_t radian);

#ifdef	__cplusplus
}
#endif

#endif
