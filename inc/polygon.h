//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_POLYGON_H
#define	UTIL_C_CRT_GEOMETRY_POLYGON_H

#include "geometry_def.h"

typedef struct GeometryRect_t {
	CCTNum_t o[3];
	CCTNum_t w_axis[3];
	CCTNum_t h_axis[3];
	CCTNum_t normal[3];
	CCTNum_t half_w;
	CCTNum_t half_h;
} GeometryRect_t;

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathTriangleGetPoint(const CCTNum_t tri[3][3], CCTNum_t u, CCTNum_t v, CCTNum_t p[3]);
__declspec_dll int mathTrianglePointUV(const CCTNum_t tri[3][3], const CCTNum_t p[3], CCTNum_t* p_u, CCTNum_t* p_v);
__declspec_dll int mathTriangleHasPoint(const CCTNum_t tri[3][3], const CCTNum_t p[3]);
__declspec_dll void mathTriangleToPolygon(const CCTNum_t tri[3][3], GeometryPolygon_t* polygon);

__declspec_dll int mathRectHasPoint(const GeometryRect_t* rect, const CCTNum_t p[3]);
__declspec_dll void mathRectVertices(const GeometryRect_t* rect, CCTNum_t p[4][3]);
__declspec_dll void mathRectToPolygon(const GeometryRect_t* rect, GeometryPolygon_t* polygon, CCTNum_t buf_points[4][3]);
__declspec_dll GeometryRect_t* mathAABBPlaneRect(const CCTNum_t o[3], const CCTNum_t half[3], unsigned int idx, GeometryRect_t* rect);
__declspec_dll GeometryRect_t* mathOBBPlaneRect(const GeometryOBB_t* obb, unsigned int idx, GeometryRect_t* rect);

__declspec_dll int mathPolygonIsConvex(const GeometryPolygon_t* polygon);
__declspec_dll GeometryPolygon_t* mathPolygonCooking(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryPolygon_t* mathPolygonDeepCopy(GeometryPolygon_t* dst, const GeometryPolygon_t* src);
__declspec_dll void mathPolygonFreeCookingData(GeometryPolygon_t* polygon);

int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
int Polygon_Contain_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2);

#ifdef	__cplusplus
}
#endif

#endif
