//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_POLYGON_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_POLYGON_COOKING_H

#include "polygon.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll int mathCookingStage1(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, CCTNum_t(**ret_v)[3], unsigned int* ret_v_cnt, unsigned int** ret_tri_indices);
__declspec_dll int mathCookingStage2(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t** ret_polygons, unsigned int* ret_polygons_cnt);
__declspec_dll int mathCookingStage3(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, unsigned int** ret_edge_indices, unsigned int* ret_edge_indices_cnt);
__declspec_dll int mathCookingStage4(const unsigned int* edge_indices, unsigned int edge_indices_cnt, unsigned int** ret_v_indices, unsigned int* ret_v_indices_cnt);

__declspec_dll GeometryPolygon_t* mathCookingPolygon(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);

#ifdef	__cplusplus
}
#endif

#endif
