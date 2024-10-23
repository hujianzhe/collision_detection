//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_POLYGON_H
#define	UTIL_C_CRT_GEOMETRY_POLYGON_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathTriangleGetPoint(const CCTNum_t tri[3][3], CCTNum_t u, CCTNum_t v, CCTNum_t p[3]);
__declspec_dll int mathTrianglePointUV(const CCTNum_t tri[3][3], const CCTNum_t p[3], CCTNum_t* p_u, CCTNum_t* p_v);
__declspec_dll int mathTriangleHasPoint(const CCTNum_t tri[3][3], const CCTNum_t p[3]);
__declspec_dll void mathTriangleToPolygon(const CCTNum_t tri[3][3], GeometryPolygon_t* polygon);

__declspec_dll int mathPolygonIsConvex(const GeometryPolygon_t* polygon, CCTNum_t epsilon);
__declspec_dll GeometryPolygon_t* mathPolygonCookingDirect(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryPolygon_t* mathPolygonCooking(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryPolygon_t* mathPolygonDeepCopy(GeometryPolygon_t* dst, const GeometryPolygon_t* src);
__declspec_dll void mathPolygonFreeCookingData(GeometryPolygon_t* polygon);

#ifdef	__cplusplus
}
#endif

#endif
