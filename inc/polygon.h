//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_POLYGON_H
#define	UTIL_C_CRT_GEOMETRY_POLYGON_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathTriangleGetPoint(const CCTNum_t tri_p0[3], const CCTNum_t tri_p1[3], const CCTNum_t tri_p2[3], CCTNum_t u, CCTNum_t v, CCTNum_t p[3]);
__declspec_dll int mathTrianglePointUV(const CCTNum_t tri_p0[3], const CCTNum_t tri_p1[3], const CCTNum_t tri_p2[3], const CCTNum_t p[3], CCTNum_t* p_u, CCTNum_t* p_v);
__declspec_dll void mathTriangleToPolygon(const CCTNum_t tri[3][3], GeometryPolygon_t* polygon);

__declspec_dll GeometryPolygon_t* mathPolygonDeepCopy(GeometryPolygon_t* dst, const GeometryPolygon_t* src);
__declspec_dll void mathPolygonFreeData(GeometryPolygon_t* polygon);

__declspec_dll int mathPolygonIsConvex(const GeometryPolygon_t* polygon);
__declspec_dll void mathPolygonEdgeNormalOuter(const GeometryPolygon_t* polygon, unsigned int edge_id, CCTNum_t edge_normal[3]);
__declspec_dll GeometryPolygonVertexAdjacentInfo_t* mathPolygonVertexAdjacentInfo(const unsigned int* edge_v_ids, unsigned int edge_v_indices_cnt, unsigned int v_id, GeometryPolygonVertexAdjacentInfo_t* info);

#ifdef	__cplusplus
}
#endif

#endif
