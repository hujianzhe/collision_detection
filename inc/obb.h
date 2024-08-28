//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_OBB_H
#define	UTIL_C_CRT_GEOMETRY_OBB_H

#include "box.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryOBB_t* mathOBBFromAABB(GeometryOBB_t* obb, const CCTNum_t o[3], const CCTNum_t half[3]);
__declspec_dll void mathOBBToAABB(const GeometryOBB_t* obb, CCTNum_t o[3], CCTNum_t half[3]);

__declspec_dll void mathOBBVertices(const GeometryOBB_t* obb, CCTNum_t v[8][3]);
__declspec_dll CCTNum_t* mathOBBVertex(const GeometryOBB_t* obb, unsigned int idx, CCTNum_t v[3]);
__declspec_dll void mathOBBClosestPointTo(const GeometryOBB_t* obb, const CCTNum_t p[3], CCTNum_t closest_p[3]);

#ifdef	__cplusplus
}
#endif

#endif
