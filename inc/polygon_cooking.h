//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_POLYGON_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_POLYGON_COOKING_H

#include "polygon.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryPolygon_t* mathPolygonCookingDirect(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryPolygon_t* mathPolygonCooking(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);

#ifdef	__cplusplus
}
#endif

#endif